from opendbc.can.packer import CANPacker
from opendbc.car import Bus
from opendbc.car.lateral import apply_driver_steer_torque_limits
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.mg.mgcan import create_cruise_buttons, create_lka_steering
from opendbc.car.mg.values import CarControllerParams, NON_EV_CAR


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.packer = CANPacker(dbc_names[Bus.pt])

    self.apply_torque_last = 0
    self.last_button_counter = -1

  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    can_sends = []

    # steering command
    if self.frame % CarControllerParams.STEER_STEP == 0:
      if CC.latActive:
        new_torque = int(round(actuators.torque * CarControllerParams.STEER_MAX))
        apply_torque = apply_driver_steer_torque_limits(new_torque, self.apply_torque_last, CS.out.steeringTorque, CarControllerParams)
      else:
        apply_torque = 0

      self.apply_torque_last = apply_torque
      can_sends.append(create_lka_steering(self.packer, (self.frame // CarControllerParams.STEER_STEP) % 16, apply_torque, CC.latActive))

    # MG HS stop and go: in Standstill Wait the ACC only moves off after RES. When openpilot's
    # planner (CC.cruiseControl.resume) and the radar's go prompt agree the lead has left,
    # send RES right after each gateway cruise switch frame, with the next counter value
    if self.CP.carFingerprint in NON_EV_CAR and CS.cruise_buttons_msg is not None:
      counter = int(CS.cruise_buttons_msg["CCSwStsAlvRC_h2HSC2"])
      if counter != self.last_button_counter:
        self.last_button_counter = counter
        if CC.cruiseControl.resume and CS.acc_go_notifier:
          can_sends.append(create_cruise_buttons(self.packer, CS.cruise_buttons_msg, (counter + 1) % 4))

    new_actuators = actuators.as_builder()
    new_actuators.torque = self.apply_torque_last / CarControllerParams.STEER_MAX
    new_actuators.torqueOutputCan = self.apply_torque_last

    self.frame += 1
    return new_actuators, can_sends
