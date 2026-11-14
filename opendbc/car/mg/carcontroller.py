from opendbc.can.packer import CANPacker
from opendbc.car import Bus
from opendbc.car.lateral import apply_driver_steer_torque_limits
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.mg.mgcan import create_cruise_buttons, create_lka_steering
from opendbc.car.mg.values import CarControllerParams, NON_EV_CAR

# cruise switch tap timing, in gateway cruise switch frames (~30 ms each)
BUTTON_PRESS_FRAMES = 4  # a real stalk press is ~6 frames, 3 is already enough
BUTTON_RETRY_FRAMES = 33  # ~1 s between taps if the ACC hasn't reacted yet


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.packer = CANPacker(dbc_names[Bus.pt])
    self.params = CarControllerParams(CP)

    self.apply_torque_last = 0
    self.last_button_counter = -1
    self.button_frames = 0

  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    can_sends = []

    # steering command
    if self.frame % self.params.STEER_STEP == 0:
      if CC.latActive:
        new_torque = int(round(actuators.torque * self.params.STEER_MAX))
        apply_torque = apply_driver_steer_torque_limits(new_torque, self.apply_torque_last, CS.out.steeringTorque, self.params)
      else:
        apply_torque = 0

      self.apply_torque_last = apply_torque
      can_sends.append(create_lka_steering(self.packer, (self.frame // self.params.STEER_STEP) % 16, apply_torque, CC.latActive))

    # MG HS cruise switches: tap a switch right after gateway cruise switch frames, with the next counter value
    # - stop and go: in Standstill Wait the ACC only moves off after RES, sent when openpilot's planner
    #   (CC.cruiseControl.resume) and the radar's go prompt agree the lead has left
    # - cancel: stop the stock ACC when openpilot disengages while it is still active
    # The ACC acts once the switch is released (seen with RES and SET): holding it (sending every frame) is
    # ignored and also masks the driver's own presses, so press for a few frames then let go
    if self.CP.carFingerprint in NON_EV_CAR and CS.cruise_buttons_msg is not None:
      counter = int(CS.cruise_buttons_msg["CCSwStsAlvRC_h2HSC2"])
      if counter != self.last_button_counter:
        self.last_button_counter = counter
        button = None
        if CC.cruiseControl.cancel:
          button = "CCSwStsCanclSwA_h2HSC2"
        elif CC.cruiseControl.resume and CS.acc_go_notifier:
          button = "CCSwStsRsmSwA_h2HSC2"

        if button is not None:
          if self.button_frames % BUTTON_RETRY_FRAMES < BUTTON_PRESS_FRAMES:
            can_sends.append(create_cruise_buttons(self.packer, CS.cruise_buttons_msg, (counter + 1) % 4, button))
          self.button_frames += 1
        else:
          self.button_frames = 0

    new_actuators = actuators.as_builder()
    new_actuators.torque = self.apply_torque_last / self.params.STEER_MAX
    new_actuators.torqueOutputCan = self.apply_torque_last

    self.frame += 1
    return new_actuators, can_sends
