import copy

from opendbc.can.parser import CANParser
from opendbc.car import Bus, create_button_events, structs
from opendbc.car.interfaces import CarStateBase
from opendbc.car.mg.values import CAR, DBC, NON_EV_CAR
from opendbc.car.common.conversions import Conversions as CV

ButtonType = structs.CarState.ButtonEvent.Type
GearShifter = structs.CarState.GearShifter

# MG HS cruise switches in GW_HSC2_FrP04, one bit each
CRUISE_BUTTONS = {
  "CCSwStsSetSwA_h2HSC2": ButtonType.setCruise,
  "CCSwStsRsmSwA_h2HSC2": ButtonType.resumeCruise,
  "CCSwStsCanclSwA_h2HSC2": ButtonType.cancel,
  "CCSwStsSpdIncSwA_h2HSC2": ButtonType.accelCruise,
  "CCSwStsSpdDecSwA_h2HSC2": ButtonType.decelCruise,
  "CCSwStsDistIncSwA_h2HSC2": ButtonType.gapAdjustCruise,
  "CCSwStsDistDecSwA_h2HSC2": ButtonType.gapAdjustCruise,
}

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
    self.cruise_buttons = dict.fromkeys(CRUISE_BUTTONS, 0)
    self.acc_go_notifier = False
    self.lka_sys_status = 0
    self.acc_state = 0

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
    if self.CP.carFingerprint == CAR.MG_HS:
      # EPS LKA control status: 0 idle, 1 following the torque request, 4 locked out after a hard driver
      # override (clears once a request is sent again, so it must not stop openpilot from requesting),
      # 3 latched fault: seen after requesting over 3 Nm, the EPS ignored all requests until the end of the drive
      ret.steerFaultPermanent = cp.vl["EPS_HSC2_FrP03"]["ChLKACtrlStsHSC2"] == 3

    # Cruise state
    acc_state = cp.vl["RADAR_HSC2_FrP00"]["ACCSysSts_RadarHSC2"]
    self.acc_state = int(acc_state)
    ret.cruiseState.speed = cp.vl["RADAR_HSC2_FrP02"]["ACCDrvrSelTrgtSpd_RadarHSC2"] * CV.KPH_TO_MS
    if non_ev:
      # MG HS (ZS-family ADAS): as in the original hs branch
      # stopping behind a lead goes Active -> 5 (Standstill Active) -> 6 (Standstill Wait, ~3 s later);
      # stay engaged there so openpilot can resume (keep in sync with safety)
      ret.cruiseState.enabled = acc_state in (2, 3, 5, 6)  # Active, Override, Standstill Active/Wait
      ret.cruiseState.available = acc_state != 0  # Off, otherwise Standby (1) or engaged
      ret.cruiseState.standstill = acc_state == 6  # Standstill Wait: needs RES or gas to go
      # the radar's own "lead has left, press RES to go" prompt
      self.acc_go_notifier = cp.vl["RADAR_HSC2_FrP02"]["ACCGoNotfr_RadarHSC2"] == 1
      # MG ZS: the cruise switch frame is optional (see get_can_parsers); only use it once it has been received,
      # so the controller never sends RES/cancel built from an empty frame
      cruise_buttons_msg = cp.vl["GW_HSC2_FrP04"]  # registers the message before ts_nanos is read
      buttons_seen = cp.ts_nanos["GW_HSC2_FrP04"]["CCSwStsAlvRC_h2HSC2"] > 0
      self.cruise_buttons_msg = copy.copy(cruise_buttons_msg) if buttons_seen else None

      # CCSwStsOnSwA_h2HSC2 is not a button but the ACC main state: it went to 0 together with ACC state 0 (Off)
      # when ACC was switched off with Cancel, and back to 1 with RES, so available above already follows it
      prev_cruise_buttons = self.cruise_buttons
      self.cruise_buttons = {sig: int(cruise_buttons_msg[sig]) for sig in CRUISE_BUTTONS}
      ret.buttonEvents = [ev for sig, btn in CRUISE_BUTTONS.items()
                          for ev in create_button_events(self.cruise_buttons[sig], prev_cruise_buttons[sig], {1: btn})]
      ret.accFaulted = cp_cam.vl["FVCM_HSC2_FrP02"]["TJAICASysFltStsHSC2"] != 0
      # the stock camera's own LKA status, from its steering frame that openpilot blocks (0 until received)
      self.lka_sys_status = int(cp_cam.vl["FVCM_HSC2_FrP03"]["LKASysStsHSC2"])
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

    # Doors: on the MG HS each door was opened and closed while parked, and each of these signals followed it
    doors = [cp.vl["GW_HSC2_BCM_FrP04"]["DrvrDoorOpenStsHSC2"],
             cp.vl["GW_HSC2_BCM_FrP04"]["FrtPsngDoorOpenStsHSC2"],
             cp.vl["GW_HSC2_BCM_FrP04"]["RLDoorOpenStsHSC2"],
             cp.vl["GW_HSC2_BCM_FrP04"]["RRDoorOpenStsHSC2"]]
    if self.CP.carFingerprint == CAR.MG_ZS:
      # MG ZS (doors opened one by one while parked): the driver door is at 47|2, BntOpenStsHSC2 in this DBC
      # (DrvrDoorOpenSts_H1_Safety in the original hs branch), and DrvrDoorOpenStsHSC2 stays 0;
      # both rear doors show on RLDoorOpenStsHSC2
      doors.append(cp.vl["GW_HSC2_BCM_FrP04"]["BntOpenStsHSC2"])
    ret.doorOpen = any(doors)

    # Blinkers
    left_stalk = cp.vl["GW_HSC2_BCM_FrP04"]["DircnIndLampSwStsHSC2"] == 1
    right_stalk = cp.vl["GW_HSC2_BCM_FrP04"]["DircnIndLampSwStsHSC2"] == 2
    if self.CP.carFingerprint == CAR.MG_ZS:
      # MG ZS: blinker lamp status, as in the mg-zs branch
      ret.leftBlinker = cp.vl["GW_HSC2_BCM_FrP04"]["BlinkerLeft"] == 1
      ret.rightBlinker = cp.vl["GW_HSC2_BCM_FrP04"]["BlinkerRight"] == 1
    elif non_ev:
      # a short "comfort" tap only shows on the stalk switch for a moment: hold it 1 s (100 frames)
      ret.leftBlinker, ret.rightBlinker = self.update_blinker_from_stalk(100, left_stalk, right_stalk)
    else:
      ret.leftBlinker, ret.rightBlinker = left_stalk, right_stalk

    # Seatbelt
    ret.seatbeltUnlatched = cp.vl["GW_HSC2_SDM_FrP00"]["DrvrSbltAtcHSC2"] != 1

    if non_ev:
      # Blindspot (RDA_HSC1_P02, only in the mg_hs DBC)
      ret.leftBlindspot = cp.vl["RDA_HSC1_P02"]["LBSDAndLCAWrnng_HS"] > 0
      ret.rightBlindspot = cp.vl["RDA_HSC1_P02"]["RBSDAndLCAWrnng_HS"] > 0

      # Stock FCW / AEB. Seen while driving: AEB, pedestrian AEB and FCW display on (2), FCW sensitivity level 2,
      # AEBMsgReq 1 ("unable to active AEB") for <1 s at ~48 km/h; 1/4/5 are only status messages, not AEB braking.
      # In the car settings menu, FCW sensitivity 1/2/3 shows in FCWSnstvtLvl and turning AEB off sets
      # AEBPedtrnDspCmd to 1; neither changes the warning or braking signals below
      ret.stockFcw = cp.vl["RADAR_HSC2_FrP02"]["FCWrnngSts_RadarHSC2"] != 0  # warning level 1-3
      ret.stockAeb = cp.vl["RADAR_HSC2_FrP02"]["AEBMsgReqHSC2"] == 2  # radar: AEB active
      if self.CP.carFingerprint == CAR.MG_HS:
        # MG ZS: SCS_HSC2_FrP24 (0x251) is not on its bus (the mg-zs branch never read it), and reading a
        # message makes it required, so CAN would never be valid ("Unknown Vehicle Variant")
        ret.stockAeb = ret.stockAeb or cp.vl["SCS_HSC2_FrP24"]["ChAEBDclReqRespHSC2"] == 1  # ABS: AEB decel request honored

      # fork: the stock radar's lead and the ACC's acceleration request, for openpilot's warning when the radar misses
      # a lead the camera sees. The lead flag followed openpilot's lead on both cars (HS route 23: 146 of 152 samples,
      # ZS routes 06/08: 347 of 357). The request matched the HS's acceleration; on the ZS 0x242 is all zeros and no
      # bus 0 signal follows the car's acceleration (the radar likely talks to the ABS on a bus the harness can't
      # see), so it is NaN there and openpilot uses the measured acceleration instead
      ret.stockRadarValid = True
      ret.stockRadarLead = cp.vl["RADAR_HSC2_FrP02"]["ACCObjDet_RadarHSC2"] == 1
      if self.CP.carFingerprint == CAR.MG_HS:
        ret.stockAccelRequest = float(cp.vl["RADAR_HSC2_FrP00"]["ACCAccReqValHSC2"])
      else:
        ret.stockAccelRequest = float('nan')

    return ret

  @staticmethod
  def get_can_parsers(CP):
    # MG ZS: the cruise switch frame (0x1E1) was not used by the mg-zs branch and may not be on this bus,
    # so its absence must not make CAN invalid
    pt_messages = [("GW_HSC2_FrP04", float('nan'))] if CP.carFingerprint == CAR.MG_ZS else []
    # the stock camera's steering frame is only used for the dashboard LKA lamp, so it is optional
    cam_messages = [("FVCM_HSC2_FrP03", float('nan'))] if CP.carFingerprint in NON_EV_CAR else []
    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], pt_messages, 0),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], cam_messages, 2),
    }
