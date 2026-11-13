from opendbc.car.crc import CRC8J1850


def calc_checksum(values):
  lka_req_toq = values['LKAReqToqHSC2'] + 0x400
  lka_req_toq_sts = values['LKAReqToqStsHSC2']
  lka_req_toq_v = values['LKAReqToqVHSC2']
  lka_alv_rc = values['LKAAlvRCHSC2']

  combined = ((lka_req_toq << 1) | (lka_req_toq_sts << 12) | lka_req_toq_v) & 0x3FFF
  with_counter = (combined + lka_alv_rc) & 0x3FFF
  checksum = ((~with_counter) + 1) & 0x3FFF

  return checksum


def create_lka_steering(packer, counter, apply_torque, active):
  values = {
    "LKAReqToqHSC2": apply_torque,
    "LKAReqToqVHSC2": 0,
    "LKAAlvRCHSC2": counter,
    "LDWLKAVbnLvlReqHSC2": 0,  # TODO: vibration level?
    "LKASysStsHSC2": 0,
    "LKAReqToqStsHSC2": active,
    "LKASysFltStsHSC2": 0,
    "LKADrvrTkovReqHSC2": 0,
    "LKAReqToqPVHSC2": 0
  }

  values["LKAReqToqPVHSC2"] = calc_checksum(values)
  return packer.make_can_msg("FVCM_HSC2_FrP03", 0, values)


def create_cruise_buttons(packer, buttons_msg, counter):
  # MG HS: copy of the gateway's cruise switch frame (GW_HSC2_FrP04) with only RES pressed
  values = {s: buttons_msg[s] for s in ("CCSwStsSwDataIntgty_h2HSC2", "CCSwStsOnSwA_h2HSC2")}
  values.update({
    "CCSwStsRsmSwA_h2HSC2": 1,
    "CCSwStsAlvRC_h2HSC2": counter,
    "CCSwStsPV_h2HSC2": 0,
  })
  # PV: all 7 bytes sum to 0 (mod 256), checked against captured frames
  dat = packer.make_can_msg("GW_HSC2_FrP04", 0, values)[1]
  values["CCSwStsPV_h2HSC2"] = -sum(dat) & 0xFF
  return packer.make_can_msg("GW_HSC2_FrP04", 0, values)


def mg_checksum(address: int, sig, d: bytearray) -> int:
  if address == 0x1EC:
    torque = ((d[0] & 0x7) << 8) | d[1]
    valid = (d[0] >> 3) & 1
    status = (d[6] >> 4) & 0x7
    counter = d[0] >> 4
    return -((torque << 1) + (status << 12) + valid + counter) & 0x7FFF

  crc = 0xFF
  for byte in d[:-1]:
    crc = CRC8J1850[crc ^ byte]
  return crc ^ 0xFF
