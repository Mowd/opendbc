import copy

from opendbc.can.parser import CANParser
from opendbc.car import Bus, structs
from opendbc.car.interfaces import CarStateBase
from opendbc.car.mg.values import DBC, NON_EV_CAR
from opendbc.car.common.conversions import Conversions as CV

GearShifter = structs.CarState.GearShifter

GEAR_MAP = {
  0: GearShifter.unknown,
  15: GearShifter.park,
  14: GearShifter.reverse,
  13: GearShifter.neutral,
  **{i: GearShifter.drive for i in range(1, 9)},
}

# TrShftLvrPos_h1HSC2 (shift lever position), used by ICE/PHEV
GEAR_MAP_NON_EV = {
  1: GearShifter.park,
  2: GearShifter.reverse,
  3: GearShifter.neutral,
  **{i: GearShifter.drive for i in range(4, 12)},
}


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.cruise_buttons_msg = None
    self.acc_go_notifier = False

  def update(self, can_parsers) -> structs.CarState:
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]
    ret = structs.CarState()
    non_ev = self.CP.carFingerprint in NON_EV_CAR

    # Vehicle speed
    ret.vEgoRaw = cp.vl["SCS_HSC2_FrP19"]["VehSpdAvgHSC2"] * CV.KPH_TO_MS
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    if non_ev:
      ret.standstill = ret.vEgoRaw < 0.01
    else:
      ret.standstill = cp.vl["SCS_HSC2_FrP24"]["VehSdslStsHSC2"] == 1

    # Gas pedal
    if non_ev:
      ret.gasPressed = cp.vl["Tester_HSC2_ECM_FrP00"]["AccelActuPosHSC2"] > 0
    else:
      ret.gasPressed = cp.vl["GW_HSC2_HCU_FrP00"]["EPTAccelActuPosHSC2"] > 0

    # Brake pedal
    if non_ev:
      # driver applied brake pressure, 75 kPa per bit; keep in sync with safety (raw > 1)
      ret.brakePressed = cp.vl["SCS_HSC2_FrP09"]["BrkPdlDrvrAppdPrsHSC2"] > 100
    else:
      ret.brakePressed = cp.vl["EHBS_HSC2_FrP00"]["BrkPdlAppdHSC2"] == 1

    # Steering wheel
    ret.steeringAngleDeg = cp.vl["SAS_HSC2_FrP00"]["StrgWhlAngHSC2"]
    ret.steeringRateDeg = cp.vl["SAS_HSC2_FrP00"]["StrgWhlAngGrdHSC2"]
    ret.steeringTorque = cp.vl["EPS_HSC2_FrP03"]["DrvrStrgDlvrdToqHSC2"]
    ret.steeringTorqueEps = cp.vl["EPS_HSC2_FrP03"]["ChLKARespToqHSC2"]
    ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > 1.0, 5)

    ret.steerFaultTemporary = cp_cam.vl["FVCM_HSC2_FrP02"]["LDWSysFltStsHSC2"] != 0

    # Cruise state
    acc_state = cp.vl["RADAR_HSC2_FrP00"]["ACCSysSts_RadarHSC2"]
    ret.cruiseState.speed = cp.vl["RADAR_HSC2_FrP02"]["ACCDrvrSelTrgtSpd_RadarHSC2"] * CV.KPH_TO_MS
    if non_ev:
      # MG HS (ZS-family ADAS): as in the original hs branch
      # stopping behind a lead goes Active -> 5 (Standstill Active) -> 6 (Standstill Wait, ~3 s later);
      # stay engaged there so openpilot can resume (keep in sync with safety)
      ret.cruiseState.enabled = acc_state in (2, 3, 5, 6)  # Active, Override, Standstill Active/Wait
      ret.cruiseState.available = True
      ret.cruiseState.standstill = acc_state == 6  # Standstill Wait: needs RES or gas to go
      # the radar's own "lead has left, press RES to go" prompt
      self.acc_go_notifier = cp.vl["RADAR_HSC2_FrP02"]["ACCGoNotfr_RadarHSC2"] == 1
      self.cruise_buttons_msg = copy.copy(cp.vl["GW_HSC2_FrP04"])
      ret.accFaulted = cp_cam.vl["FVCM_HSC2_FrP02"]["TJAICASysFltStsHSC2"] != 0
    else:
      ret.cruiseState.enabled = acc_state in (2, 3)  # Active, Override
      ret.cruiseState.available = acc_state != 0
      ret.cruiseState.standstill = False
      ret.accFaulted = cp.vl["RADAR_HSC2_FrP00"]["ACCSysFltSts_SCSHSC2"] != 0

    # Gear
    if non_ev:
      ret.gearShifter = GEAR_MAP_NON_EV.get(int(cp.vl["GW_HSC2_ECM_FrP04"]["TrShftLvrPos_h1HSC2"]), GearShifter.unknown)
    else:
      ret.gearShifter = GEAR_MAP.get(int(cp.vl["GW_HSC2_ECM_FrP04"]["TrEstdGearHSC2"]), GearShifter.unknown)

    # Doors
    if non_ev:
      ret.doorOpen = any([cp.vl["GW_HSC2_BCM_FrP04"]["DrvrDoorOpenSts_H1_Safety"],
                          cp.vl["GW_HSC2_BCM_FrP04"]["FrtPsngDoorOpenStsHSC2"]])
    else:
      ret.doorOpen = any([cp.vl["GW_HSC2_BCM_FrP04"]["DrvrDoorOpenStsHSC2"],
                          cp.vl["GW_HSC2_BCM_FrP04"]["FrtPsngDoorOpenStsHSC2"],
                          cp.vl["GW_HSC2_BCM_FrP04"]["RLDoorOpenStsHSC2"],
                          cp.vl["GW_HSC2_BCM_FrP04"]["RRDoorOpenStsHSC2"]])

    # Blinkers
    ret.leftBlinker = cp.vl["GW_HSC2_BCM_FrP04"]["DircnIndLampSwStsHSC2"] == 1
    ret.rightBlinker = cp.vl["GW_HSC2_BCM_FrP04"]["DircnIndLampSwStsHSC2"] == 2

    # Seatbelt
    ret.seatbeltUnlatched = cp.vl["GW_HSC2_SDM_FrP00"]["DrvrSbltAtcHSC2"] != 1

    if non_ev:
      # Blindspot (RDA_HSC1_P02, only in the mg_hs DBC)
      ret.leftBlindspot = cp.vl["RDA_HSC1_P02"]["LBSDAndLCAWrnng_HS"] > 0
      ret.rightBlindspot = cp.vl["RDA_HSC1_P02"]["RBSDAndLCAWrnng_HS"] > 0

    return ret

  @staticmethod
  def get_can_parsers(CP):
    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 0),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 2),
    }
