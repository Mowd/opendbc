from opendbc.can.packer import CANPacker
from opendbc.car import Bus
from opendbc.car.lateral import apply_driver_steer_torque_limits
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.mg.mgcan import create_cruise_buttons, create_lka_steering
from opendbc.car.mg.speed_buttons import SpeedButtons
from opendbc.car.mg.values import CarControllerParams, NON_EV_CAR

# cruise switch tap timing, in gateway cruise switch frames (~30 ms each)
BUTTON_PRESS_FRAMES = 4  # a real stalk press is ~6 frames, 3 is already enough
BUTTON_RETRY_FRAMES = 33  # ~1 s between taps if the ACC hasn't reacted yet
# braking: the stock ACC drops to Standby by itself, but up to ~0.2 s after openpilot sees the brake pressure
BRAKE_CANCEL_HOLD_FRAMES = 100  # 1 s at 100 Hz
# EPS lockout. ChLKACtrlStsHSC2: 0 released, 1 following the request, 4 locked out. Locked out or stuck, the EPS gives
# no torque while openpilot still shows engaged, so drop the request until the EPS has released (0), then request again.
# - MG ZS route 3d: after the driver steered against the request the EPS locked out (4) and stayed so for 18-120 s while
#   openpilot kept requesting; each time it released 0.03-2 s after the request was dropped and followed the next one.
# - It locks out in two steps: first it freezes its output (ChLKARespToqHSC2 stays at one value, no longer following),
#   then locks out exactly 1.9 s later unless that ends first (ZS: 8 freezes, the 4 that lasted 1.9 s all locked out).
#   ZS route 41: a freeze also locked out after the driver had let go, and in a curve with the request at 3 Nm.
# - ZS route 41: dropping the request for a fixed 0.2 s was too short. Once the EPS ramped its frozen output down more
#   slowly (1.45 Nm in 0.3 s) the request came back before it had released, and it stayed at 1 with no output at all
#   (7-29 s, three times), so wait for the release.
# Triggers: locked out; frozen for 0.24 s with the output 0.3 Nm or more and 0.25 Nm away from the request; following
# with exactly no output for 0.5 s while requesting 0.5 Nm or more. Replays: ZS caught all 9 freezes and the 3 stuck
# periods, no other triggers in ZS (31 min) or MG HS (51 min) steering. Most MG HS lockouts also released without a
# request, a few only once one was sent again: if the EPS hasn't released in EPS_RELEASE_MAX_FRAMES, request for
# EPS_RETRY_FRAMES before dropping it again.
EPS_RELEASED = 0
EPS_FOLLOWING = 1
EPS_LOCKOUT = 4
EPS_RELEASE_MIN_FRAMES = 5  # steering frames (0.1 s at 50 Hz) without a request
EPS_RELEASE_MAX_FRAMES = 100  # 2 s
EPS_RETRY_FRAMES = 50  # 1 s
EPS_FREEZE_FRAMES = 12
EPS_FREEZE_MIN_OUTPUT = 0.3  # Nm
EPS_FREEZE_GAP = 25  # request vs output, in request units (0.01 Nm)
EPS_STUCK_FRAMES = 25
EPS_STUCK_REQUEST = 50  # request units

class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.packer = CANPacker(dbc_names[Bus.pt])
    self.params = CarControllerParams(CP)

    self.apply_torque_last = 0
    self.last_button_counter = -1
    self.button_frames = 0
    self.last_brake_frame = -BRAKE_CANCEL_HOLD_FRAMES
    self.speed_buttons = SpeedButtons()
    self.eps_releasing = False
    self.eps_release_frames = 0
    self.eps_retry_frames = 0
    self.eps_resp_last = 0.0
    self.eps_frozen_frames = 0
    self.eps_stuck_frames = 0

  def eps_request_ok(self, CS, lat_active: bool) -> bool:
    """Every steering frame: False while the request is dropped for the EPS to release (see EPS_LOCKOUT)."""
    resp = CS.out.steeringTorqueEps
    sts = CS.eps_ctrl_sts
    self.eps_frozen_frames = self.eps_frozen_frames + 1 if abs(resp - self.eps_resp_last) < 0.005 else 0
    self.eps_resp_last = resp
    stuck = sts == EPS_FOLLOWING and abs(resp) < 0.005 and abs(self.apply_torque_last) >= EPS_STUCK_REQUEST
    self.eps_stuck_frames = self.eps_stuck_frames + 1 if stuck else 0

    if self.eps_releasing:
      self.eps_release_frames += 1
      released = sts == EPS_RELEASED and self.eps_release_frames >= EPS_RELEASE_MIN_FRAMES
      timed_out = self.eps_release_frames >= EPS_RELEASE_MAX_FRAMES
      if not (released or timed_out):
        return False
      self.eps_releasing = False
      self.eps_retry_frames = EPS_RETRY_FRAMES if timed_out else 0
      self.eps_frozen_frames = 0
      self.eps_stuck_frames = 0
      return True

    if not lat_active:
      self.eps_retry_frames = 0
      return True
    if self.eps_retry_frames > 0:
      self.eps_retry_frames -= 1
      return True

    frozen = (sts == EPS_FOLLOWING and self.eps_frozen_frames >= EPS_FREEZE_FRAMES and abs(resp) >= EPS_FREEZE_MIN_OUTPUT and
              abs(self.apply_torque_last - resp * 100) > EPS_FREEZE_GAP)
    if sts == EPS_LOCKOUT or frozen or self.eps_stuck_frames >= EPS_STUCK_FRAMES:
      self.eps_releasing = True
      self.eps_release_frames = 0
      return False
    return True

  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    can_sends = []

    # steering command
    if self.frame % self.params.STEER_STEP == 0:
      steer = CC.latActive
      if self.CP.carFingerprint in NON_EV_CAR:
        steer = self.eps_request_ok(CS, CC.latActive) and steer

      if steer:
        new_torque = int(round(actuators.torque * self.params.STEER_MAX))
        apply_torque = apply_driver_steer_torque_limits(new_torque, self.apply_torque_last, CS.out.steeringTorque, self.params)
      else:
        apply_torque = 0

      self.apply_torque_last = apply_torque

      # MG HS: keep the dashboard LKA lamp. Active (2) while openpilot steers, otherwise what the stock camera sends
      # (Standby 1 on the MG HS, 0 when LKA is turned off in the car settings)
      lka_status = 0
      if self.CP.carFingerprint in NON_EV_CAR:
        lka_status = 2 if CC.latActive else CS.lka_sys_status
      can_sends.append(create_lka_steering(self.packer, (self.frame // self.params.STEER_STEP) % 16, apply_torque,
                                           steer, lka_status))

    # MG HS cruise switches: tap a switch right after gateway cruise switch frames, with the next counter value
    # - stop and go: in Standstill Wait the ACC only moves off after RES, sent when openpilot's planner
    #   (CC.cruiseControl.resume) and the radar's go prompt agree the lead has left
    # - cancel: stop the stock ACC when openpilot disengages while it is still active
    # The ACC acts once the switch is released (seen with RES and SET): holding it (sending every frame) is
    # ignored and also masks the driver's own presses, so press for a few frames then let go
    # - no cancel while (and shortly after) the driver brakes: the ACC is already leaving, and a Cancel tap would
    #   turn braking into a stalk cancel (Active -> 4 Brake Only -> Standby instead of Active -> Standby)
    if CS.out.brakePressed:
      self.last_brake_frame = self.frame
    braking = self.frame - self.last_brake_frame < BRAKE_CANCEL_HOLD_FRAMES

    if self.CP.carFingerprint in NON_EV_CAR and CS.cruise_buttons_msg is not None:
      # fork: curve speed, lower the stock ACC set speed with -/+ taps (see speed_buttons.py)
      set_kph = CS.out.cruiseState.speed * CV.MS_TO_KPH
      driver_pressing = any(CS.cruise_buttons.values())
      self.speed_buttons.update(self.frame, CC.enabled, CS.acc_state, set_kph,
                                CC.cruiseControl.speedTarget * CV.MS_TO_KPH, driver_pressing, CS.out.vEgo * CV.MS_TO_KPH)

      counter = int(CS.cruise_buttons_msg["CCSwStsAlvRC_h2HSC2"])
      if counter != self.last_button_counter:
        self.last_button_counter = counter
        button = None
        if CC.cruiseControl.cancel and not braking:
          button = "CCSwStsCanclSwA_h2HSC2"
        elif CC.cruiseControl.resume and CS.acc_go_notifier:
          button = "CCSwStsRsmSwA_h2HSC2"

        if button is not None:
          self.speed_buttons.reset_tap()
          if self.button_frames % BUTTON_RETRY_FRAMES < BUTTON_PRESS_FRAMES:
            can_sends.append(create_cruise_buttons(self.packer, CS.cruise_buttons_msg, (counter + 1) % 4, button))
          self.button_frames += 1
        else:
          self.button_frames = 0
          if driver_pressing:
            # never press on top of the driver
            self.speed_buttons.reset_tap()
          else:
            speed_button = self.speed_buttons.button(set_kph)
            if speed_button is not None:
              can_sends.append(create_cruise_buttons(self.packer, CS.cruise_buttons_msg, (counter + 1) % 4, speed_button))

    new_actuators = actuators.as_builder()
    new_actuators.torque = self.apply_torque_last / self.params.STEER_MAX
    new_actuators.torqueOutputCan = self.apply_torque_last

    self.frame += 1
    return new_actuators, can_sends
