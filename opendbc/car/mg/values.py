from dataclasses import dataclass, field
from enum import IntFlag

from opendbc.car import Bus, CarSpecs, DbcDict, PlatformConfig, Platforms
from opendbc.car.docs_definitions import CarHarness, CarDocs, CarParts
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, StdQueries


@dataclass
class MgCarDocs(CarDocs):
  package: str = "All"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.vw_a]))


@dataclass
class MgPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {Bus.pt: 'mg'})


class CAR(Platforms):
  MG_5_EV = MgPlatformConfig([MgCarDocs("MG 5 EV 2021")], CarSpecs(wheelbase=2.66, steerRatio=15.8))
  # MG HS shares its ADAS (camera/radar) with the MG ZS, not the MG 5 EV, so it follows the original hs branch:
  # its own DBC name means the MG 5 EV CAN checksums are not validated
  MG_HS = MgPlatformConfig(
    [MgCarDocs("MG HS PHEV 2023")],
    # wheelbase: 2720 mm (MG Taiwan / UK spec sheets)
    # steerRatio: not published; learned 16.6-17.0 by paramsd over 30 segments, 16.8 from yaw rate below 5 m/s,
    # full lock is ~480 deg each side
    CarSpecs(wheelbase=2.72, steerRatio=16.8),
    {Bus.pt: 'mg_hs_generated'},
  )
  # MG ZS 2025 (ICE): same ADAS and CAN layout as the MG HS, parameters from the mg-zs branch
  MG_ZS = MgPlatformConfig(
    [MgCarDocs("MG ZS 2025")],
    CarSpecs(wheelbase=2.58, steerRatio=15.8),
    {Bus.pt: 'mg_hs_generated'},
  )


FW_QUERY_CONFIG = FwQueryConfig(
  fw_version_regex=br"[\x10\x11][\x00-\xff]{3}\x01",
  requests=[
    # MG 5 EV: through the OBD port
    Request(
      [StdQueries.TESTER_PRESENT_REQUEST, StdQueries.MANUFACTURER_ECU_HARDWARE_NUMBER_REQUEST],
      [StdQueries.TESTER_PRESENT_RESPONSE, StdQueries.MANUFACTURER_ECU_HARDWARE_NUMBER_RESPONSE],
      bus=1,
    ),
    # MG HS: the EPS, camera and radar only answer on the ADAS CAN at the harness, not through the OBD port
    Request(
      [StdQueries.TESTER_PRESENT_REQUEST, StdQueries.MANUFACTURER_ECU_HARDWARE_NUMBER_REQUEST],
      [StdQueries.TESTER_PRESENT_RESPONSE, StdQueries.MANUFACTURER_ECU_HARDWARE_NUMBER_RESPONSE],
      bus=0,
    ),
  ],
)

class CarControllerParams:
  STEER_STEP = 2  # FVCM_HSC2_FrP03 message frequency 50Hz
  STEER_MAX = 300
  STEER_DELTA_UP = 6       # Max torque reached in 1.00s (STEER_MAX / (50Hz * 1.00))
  STEER_DELTA_DOWN = 10    # Min torque reached in 0.60s (STEER_MAX / (50Hz * 0.60))
  STEER_DRIVER_ALLOWANCE = 100  # allowed driver torque before start limiting
  STEER_DRIVER_MULTIPLIER = 2  # weight driver torque
  STEER_DRIVER_FACTOR = 100

  def __init__(self, CP):
    if CP.carFingerprint in NON_EV_CAR:
      # MG HS: ramp torque a bit faster, still within the ISO 11270 jerk limits (keep in sync with safety)
      self.STEER_DELTA_UP = 7     # Max torque reached in ~0.86s
      self.STEER_DELTA_DOWN = 11  # Min torque reached in ~0.55s
    if CP.carFingerprint == CAR.MG_ZS:
      # MG ZS: the mg-zs branch used 8/8; 8 up is over the ISO 11270 jerk limit (3.3 > 3.0 m/s^3), so keep 7 up
      self.STEER_DELTA_DOWN = 8   # Min torque reached in 0.75s


class MgSafetyFlags(IntFlag):
  NON_EV = 1  # ICE/PHEV: gas from ECM (0xc9), brake from driver brake pressure (0x214)


NON_EV_CAR = {CAR.MG_HS, CAR.MG_ZS}

DBC = CAR.create_dbc_map()
