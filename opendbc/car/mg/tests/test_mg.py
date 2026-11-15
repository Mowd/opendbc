from opendbc.can.dbc import DBC
from opendbc.car import Bus
from opendbc.car.mg.interface import CarInterface
from opendbc.car.mg.values import CAR, DBC as MG_DBC

# Messages the mg-zs branch read on the MG ZS, which ran there without CAN errors.
# Every message CarState reads becomes required, so the MG ZS must not read anything else
# (except optional ones), or CAN is never valid and openpilot shows "Unknown Vehicle Variant".
MG_ZS_MESSAGES = {
  0: {"SCS_HSC2_FrP19", "Tester_HSC2_ECM_FrP00", "SCS_HSC2_FrP09", "SAS_HSC2_FrP00", "EPS_HSC2_FrP03",
      "RADAR_HSC2_FrP00", "RADAR_HSC2_FrP02", "GW_HSC2_ECM_FrP04", "GW_HSC2_BCM_FrP04", "GW_HSC2_SDM_FrP00",
      "RDA_HSC1_P02"},
  2: {"FVCM_HSC2_FrP02"},
}


class TestMgZs:
  def test_can_valid_with_mg_zs_messages(self):
    CP = CarInterface.get_non_essential_params(CAR.MG_ZS)
    CI = CarInterface(CP)
    dbc = DBC(MG_DBC[CAR.MG_ZS][Bus.pt])
    frames = [(dbc.name_to_msg[name].address, bytes(8), bus) for bus, names in MG_ZS_MESSAGES.items() for name in names]

    valid = []
    for i in range(300):  # 3 s at 100 Hz
      valid.append(CI.update([(i * 10_000_000, frames)]).canValid)
    assert all(valid[10:]), "CAN not valid with only the mg-zs messages"

    for cp in CI.can_parsers.values():
      required = {s.name for s in cp.message_states.values() if not s.ignore_alive}
      assert required <= MG_ZS_MESSAGES.get(cp.bus, set()), f"bus {cp.bus} requires {required - MG_ZS_MESSAGES.get(cp.bus, set())}"
