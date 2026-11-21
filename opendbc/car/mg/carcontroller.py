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
# EPS lockout (ChLKACtrlStsHSC2 4, after the driver steers against the request): the EPS ignores requests and reports
# no torque while openpilot still shows engaged. MG ZS route 3d: 4 lockouts, the EPS stayed locked for 18-120 s while
# openpilot kept requesting (up to the full 3 Nm, nothing steered); each one cleared 0.03-2 s after the request was
# dropped, and the next request was followed. Most MG HS lockouts also cleared without a request, a few only once one
# was sent again. So pause the request, then try again, until the EPS follows.
EPS_LOCKOUT = 4
EPS_PAUSE_FRAMES = 10  # steering frames (0.2 s at 50 Hz) without a request
EPS_RETRY_FRAMES = 50  # steering frames (1 s) of requesting before pausing again


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
    self.eps_pause_frames = 0
    self.eps_retry_frames = 0

  def eps_lockout_ok(self, locked: bool) -> bool:
    """Every steering frame: False while the request is paused for an EPS lockout (see EPS_LOCKOUT)."""
    if self.eps_pause_frames > 0:
      self.eps_pause_frames -= 1
      return False
    if not locked:
      self.eps_retry_frames = 0
      return True
    if self.eps_retry_frames == 0:
      # just locked out, or still locked after requesting again for EPS_RETRY_FRAMES: pause
      self.eps_pause_frames = EPS_PAUSE_FRAMES - 1
      self.eps_retry_frames = EPS_RETRY_FRAMES
      return False
    self.eps_retry_frames -= 1
    return True

  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    can_sends = []

    # steering command
    if self.frame % self.params.STEER_STEP == 0:
      steer = CC.latActive
      if self.CP.carFingerprint in NON_EV_CAR:
        steer = steer and self.eps_lockout_ok(CC.latActive and CS.eps_ctrl_sts == EPS_LOCKOUT)

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
