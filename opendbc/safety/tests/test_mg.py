#!/usr/bin/env python3
import unittest

from opendbc.car.structs import CarParams
from opendbc.car.mg.values import MgSafetyFlags
import opendbc.safety.tests.common as common
from opendbc.safety.tests.libsafety import libsafety_py


class TestMGSafety(common.CarSafetyTest, common.DriverTorqueSteeringSafetyTest):

  DBC = "mg"
  SAFETY_MODEL = CarParams.SafetyModel.mg

  TX_MSGS = [[0x1fd, 0], ]
  RELAY_MALFUNCTION_ADDRS = {0: (0x1fd,)}
  FWD_BLACKLISTED_ADDRS = {2: [0x1fd,]}

  MAX_RATE_UP = 6
  MAX_RATE_DOWN = 10
  MAX_TORQUE_LOOKUP = [0], [300]
  MAX_RT_DELTA = 125

  DRIVER_TORQUE_ALLOWANCE = 100
  DRIVER_TORQUE_FACTOR = 2

  def setUp(self):
    super().setUp()
    self.counters = {addr: 0 for addr in (0x1b6, 0x1ec, 0x23c, 0x242)}
    self.gas_counter = 0

  def _counter(self, addr):
    counter = self.counters[addr]
    self.counters[addr] = (counter + 1) % 16
    return counter

  def _torque_cmd_msg(self, torque, steer_req=1):
    values = {"LKAReqToqHSC2": torque, "LKAReqToqStsHSC2": steer_req}
    return self.packer.make_can_msg_safety("FVCM_HSC2_FrP03", 0, values)

  def _speed_msg(self, speed):
    values = {"VehSpdAvgHSC2": speed * 3.6, "VehSpdAvgAlvRCHSC2": self._counter(0x23c)}
    return self.packer.make_can_msg_safety("SCS_HSC2_FrP19", 0, values)

  def _torque_driver_msg(self, torque):
    values = {"DrvrStrgDlvrdToqHSC2": torque * 0.01, "ChLKAAlvRCHSC2": self._counter(0x1ec)}
    return self.packer.make_can_msg_safety("EPS_HSC2_FrP03", 0, values)

  def _user_brake_msg(self, brake):
    values = {"BrkPdlAppdHSC2": 1 if brake else 0, "BrkPdlAppdRCHSC2": self._counter(0x1b6)}
    return self.packer.make_can_msg_safety("EHBS_HSC2_FrP00", 0, values)

  def _user_gas_msg(self, gas):
    values = {"EPTAccelActuPosHSC2": 100 if gas else 0}
    msg = self.packer.make_can_msg_safety("GW_HSC2_HCU_FrP00", 0, values)
    msg[0].data[5] = (msg[0].data[5] & 0x0F) | ((self.gas_counter % 16) << 4)
    self.gas_counter += 1
    return msg

  def _pcm_status_msg(self, enable):
    return self._acc_state_msg(2 if enable else 1)

  def _acc_state_msg(self, state):
    values = {"ACCSysSts_RadarHSC2": state, "ACCSysAlvRlngCtr_SCSHSC2": self._counter(0x242)}
    return self.packer.make_can_msg_safety("RADAR_HSC2_FrP00", 0, values)

  def test_standstill_not_engaged(self):
    # MG 5 EV: the standstill ACC states are not verified, they don't count as engaged
    for state in (5, 6):
      self.safety.set_controls_allowed(True)
      self._rx(self._acc_state_msg(2))
      self._rx(self._acc_state_msg(state))
      self.assertFalse(self.safety.get_controls_allowed())

  def test_gas_counter(self):
    self._reset_safety_hooks()
    for _ in range(16):
      self.assertTrue(self._rx(self._user_gas_msg(0)))

    msg = self._user_gas_msg(0)
    for _ in range(common.MAX_WRONG_COUNTERS + 1):
      valid = self._rx(msg)
    self.assertFalse(valid)

  def test_rx_checksums(self):
    # Captured EPS frame: the PV field is 0x37f5; byte 7 is unused.
    dat = bytes.fromhex("b40037f553f54000")
    values = {"ChLKAAlvRCHSC2": 11, "ChLKACtrlStsHSC2": 4, "ChLKARespToqHSC2": 0}
    packed = self.packer.make_can_msg_safety("EPS_HSC2_FrP03", 0, values)
    self.assertEqual(bytes(packed[0].data)[2:4], dat[2:4])

    for byte, bit in ((0, 0), (0, 3), (0, 4), (1, 0), (2, 0), (3, 0), (6, 4)):
      self._reset_safety_hooks()
      self.assertTrue(self._rx(libsafety_py.make_CANPacket(0x1ec, 0, dat)))
      corrupt = bytearray(dat)
      corrupt[byte] ^= 1 << bit
      self.assertFalse(self._rx(libsafety_py.make_CANPacket(0x1ec, 0, corrupt)))

    for make_msg in (self._speed_msg, self._torque_driver_msg, self._user_brake_msg, self._pcm_status_msg):
      self._reset_safety_hooks()
      self.assertTrue(self._rx(make_msg(0)))

      msg = make_msg(0)
      msg[0].data[3 if make_msg == self._torque_driver_msg else 7] ^= 0xff
      self.assertFalse(self._rx(msg))


class TestMGNonEVSafety(TestMGSafety):
  # ICE/PHEV (MG HS PHEV): gas from ECM, brake from driver brake pressure or the HCU brake switch
  SAFETY_PARAM = MgSafetyFlags.NON_EV | MgSafetyFlags.PHEV

  TX_MSGS = [[0x1fd, 0], [0x1e1, 0]]

  MAX_RATE_UP = 7
  MAX_RATE_DOWN = 11

  # GW_HSC2_FrP04 frames captured on an MG HS PHEV: On switch held, counter 0
  BUTTONS_IDLE = bytes.fromhex("40c00000000000")
  BUTTONS_RESUME = bytes.fromhex("60a00000000000")
  BUTTONS_CANCEL = bytes.fromhex("c0400000000000")

  def _button_msg(self, dat):
    return libsafety_py.make_CANPacket(0x1e1, 0, dat)

  def test_standstill_not_engaged(self):
    pass

  def test_standstill_engaged(self):
    # Active -> Standstill Active -> Standstill Wait keeps controls allowed
    self.safety.set_controls_allowed(False)
    self._rx(self._acc_state_msg(1))
    self._rx(self._acc_state_msg(2))
    self.assertTrue(self.safety.get_controls_allowed())
    for state in (5, 6, 2):
      self._rx(self._acc_state_msg(state))
      self.assertTrue(self.safety.get_controls_allowed())
    self._rx(self._acc_state_msg(1))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_resume_button(self):
    for controls_allowed in (True, False):
      self.safety.set_controls_allowed(controls_allowed)
      self.assertEqual(controls_allowed, self._tx(self._button_msg(self.BUTTONS_RESUME)))
      self.assertFalse(self._tx(self._button_msg(self.BUTTONS_IDLE)))

  def test_speed_buttons(self):
    # fork: SpdDec / SpdInc alone while engaged (curve speed), never when not engaged
    for bit in (2, 3):
      dat = bytearray(self.BUTTONS_IDLE)
      dat[0] |= 1 << bit
      for controls_allowed in (True, False):
        self.safety.set_controls_allowed(controls_allowed)
        self.assertEqual(controls_allowed, self._tx(self._button_msg(bytes(dat))), bit)
    # both at once is not a real press
    dat = bytearray(self.BUTTONS_IDLE)
    dat[0] |= 0x0C
    self.safety.set_controls_allowed(True)
    self.assertFalse(self._tx(self._button_msg(bytes(dat))))

  def test_cancel_button(self):
    # cancel is always allowed, so openpilot can stop the stock ACC after disengaging
    for controls_allowed in (True, False):
      self.safety.set_controls_allowed(controls_allowed)
      self.assertTrue(self._tx(self._button_msg(self.BUTTONS_CANCEL)))

  def test_other_buttons_blocked(self):
    for controls_allowed in (True, False):
      self.safety.set_controls_allowed(controls_allowed)
      # SpdDec, SpdInc, Set, Rsm, Cancel (byte 0) and DistDec, DistInc (byte 2), added to RES or Cancel
      for byte, bit in ((0, 2), (0, 3), (0, 4), (0, 5), (0, 7), (2, 2), (2, 3)):
        for base in (self.BUTTONS_RESUME, self.BUTTONS_CANCEL):
          dat = bytearray(base)
          if dat[byte] & (1 << bit):
            continue
          dat[byte] |= 1 << bit
          self.assertFalse(self._tx(self._button_msg(bytes(dat))), (byte, bit, base.hex()))
      # Set alone, distance switches alone
      for byte, bit in ((0, 4), (2, 2), (2, 3)):
        dat = bytearray(self.BUTTONS_IDLE)
        dat[byte] |= 1 << bit
        self.assertFalse(self._tx(self._button_msg(bytes(dat))), (byte, bit))
      # SpdDec / SpdInc with a distance switch
      for bit in (2, 3):
        dat = bytearray(self.BUTTONS_IDLE)
        dat[0] |= 1 << bit
        dat[2] |= 1 << 2
        self.assertFalse(self._tx(self._button_msg(bytes(dat))), bit)

  def setUp(self):
    super().setUp()
    self.brake_counter = 0

  def _user_brake_msg(self, brake):
    values = {"BrkPdlDrvrAppdPrsHSC2": 150 if brake else 0, "BrkPdlDrvrAppdPrsAlvRCHSC2": self.brake_counter}
    self.brake_counter = (self.brake_counter + 1) % 4
    return self.packer.make_can_msg_safety("SCS_HSC2_FrP09", 0, values)

  def _user_gas_msg(self, gas):
    values = {"AccelActuPosHSC2": 100 if gas else 0}
    msg = self.packer.make_can_msg_safety("Tester_HSC2_ECM_FrP00", 0, values)
    # byte 7 is the XOR of bytes 0-6 (not named in the DBC)
    xor = 0
    for b in bytes(msg[0].data)[:7]:
      xor ^= b
    msg[0].data[7] = xor
    return msg

  def _brake_switch_msg(self, on):
    msg = self.packer.make_can_msg_safety("GW_HSC2_HCU_FrP00", 0, {"EPTBrkPdlDscrtInptStsHSC2": 1 if on else 0})
    # byte 7 is the XOR of bytes 0-6 (not named in the DBC)
    xor = 0
    for b in bytes(msg[0].data)[:7]:
      xor ^= b
    msg[0].data[7] = xor
    return msg

  def test_brake_switch(self):
    # a light touch closes the brake switch (and cancels the stock ACC) before the pressure reaches 100 kPa
    self.assertTrue(self._rx(self._brake_switch_msg(True)))
    self.assertTrue(self.safety.get_brake_pressed_prev())
    self.assertTrue(self._rx(self._user_brake_msg(False)))
    self.assertTrue(self.safety.get_brake_pressed_prev())  # still pressed: the switch is still closed
    self.assertTrue(self._rx(self._brake_switch_msg(False)))
    self.assertFalse(self.safety.get_brake_pressed_prev())
    # and pressure alone still counts
    self.assertTrue(self._rx(self._user_brake_msg(True)))
    self.assertTrue(self.safety.get_brake_pressed_prev())

  def test_brake_switch_disengages(self):
    self.safety.set_controls_allowed(True)
    self._rx(self._speed_msg(10))
    self._rx(self._brake_switch_msg(True))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_brake_threshold(self):
    # a single 75 kPa bit is noise, not a pressed pedal
    self._rx(self.packer.make_can_msg_safety("SCS_HSC2_FrP09", 0, {"BrkPdlDrvrAppdPrsHSC2": 75, "BrkPdlDrvrAppdPrsAlvRCHSC2": 0}))
    self.assertFalse(self.safety.get_brake_pressed_prev())
    self._rx(self.packer.make_can_msg_safety("SCS_HSC2_FrP09", 0, {"BrkPdlDrvrAppdPrsHSC2": 150, "BrkPdlDrvrAppdPrsAlvRCHSC2": 1}))
    self.assertTrue(self.safety.get_brake_pressed_prev())

  def test_ev_pedal_msgs_ignored(self):
    # EV gas/brake messages must not affect the NON_EV safety state
    self._rx(self.packer.make_can_msg_safety("GW_HSC2_HCU_FrP00", 0, {"EPTAccelActuPosHSC2": 100}))
    self.assertFalse(self.safety.get_gas_pressed_prev())
    self._rx(self.packer.make_can_msg_safety("EHBS_HSC2_FrP00", 0, {"BrkPdlAppdHSC2": 1}))
    self.assertFalse(self.safety.get_brake_pressed_prev())

  def test_gas_counter(self):
    # the ECM gas frame's counter skips on the car, so it isn't checked
    self._reset_safety_hooks()
    msg = self._user_gas_msg(0)
    for _ in range(common.MAX_WRONG_COUNTERS + 2):
      self.assertTrue(self._rx(msg))

  def test_rx_counters(self):
    # speed, EPS, radar (4 bit) and brake pressure (2 bit) counters are checked
    for make_msg in (self._speed_msg, self._torque_driver_msg, self._pcm_status_msg, self._user_brake_msg):
      self._reset_safety_hooks()
      for _ in range(8):
        self.assertTrue(self._rx(make_msg(0)))
      msg = make_msg(0)
      for _ in range(common.MAX_WRONG_COUNTERS + 1):
        valid = self._rx(msg)
      self.assertFalse(valid, make_msg.__name__)

  def test_rx_checksums(self):
    # MG HS: speed, radar (CRC-8 in byte 7), EPS (PV in bytes 2-3), ECM gas and HCU brake switch (XOR in byte 7)
    # checksums are checked
    for make_msg in (self._speed_msg, self._torque_driver_msg, self._user_gas_msg, self._pcm_status_msg, self._brake_switch_msg):
      self._reset_safety_hooks()
      self.assertTrue(self._rx(make_msg(0)), make_msg.__name__)
      msg = make_msg(0)
      msg[0].data[3 if make_msg == self._torque_driver_msg else 7] ^= 0xff
      self.assertFalse(self._rx(msg), make_msg.__name__)


class TestMGHSICESafety(TestMGNonEVSafety):
  # MG HS 1.5 ICE: no HCU, so no brake switch frame (0xaf): brake from the pressure only
  SAFETY_PARAM = MgSafetyFlags.NON_EV

  def test_brake_switch(self):
    self._rx(self._brake_switch_msg(True))
    self.assertFalse(self.safety.get_brake_pressed_prev())
    self.assertTrue(self._rx(self._user_brake_msg(True)))
    self.assertTrue(self.safety.get_brake_pressed_prev())

  def test_brake_switch_disengages(self):
    pass

  def test_rx_checksums(self):
    for make_msg in (self._speed_msg, self._torque_driver_msg, self._user_gas_msg, self._pcm_status_msg):
      self._reset_safety_hooks()
      self.assertTrue(self._rx(make_msg(0)), make_msg.__name__)
      msg = make_msg(0)
      msg[0].data[3 if make_msg == self._torque_driver_msg else 7] ^= 0xff
      self.assertFalse(self._rx(msg), make_msg.__name__)


if __name__ == "__main__":
  unittest.main()
