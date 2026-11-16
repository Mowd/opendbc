from opendbc.car import get_safety_config, structs
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.mg.carcontroller import CarController
from opendbc.car.mg.carstate import CarState
from opendbc.car.mg.values import CAR, NON_EV_CAR, MgFlags, MgSafetyFlags


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "mg"
    ret.dashcamOnly = candidate not in NON_EV_CAR

    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.mg)]
    if candidate in NON_EV_CAR:
      ret.safetyConfigs[0].safetyParam |= MgSafetyFlags.NON_EV.value
    # MG HS PHEV or 1.5 ICE: only the PHEV has the HCU, whose brake switch frame (0xAF) the ICE doesn't send
    if candidate == CAR.MG_HS and 0xaf in fingerprint[0]:
      ret.flags |= MgFlags.PHEV.value
      ret.safetyConfigs[0].safetyParam |= MgSafetyFlags.PHEV.value

    ret.steerActuatorDelay = 0.3
    CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    ret.steerControlType = structs.CarParams.SteerControlType.torque
    ret.radarUnavailable = True
    ret.alphaLongitudinalAvailable = False

    return ret
