from opendbc.car.structs import CarParams
from opendbc.car.mg.values import CAR

Ecu = CarParams.Ecu

FW_VERSIONS = {
  CAR.MG_5_EV: {
    (Ecu.eps, 0x721, None): [
      b'\x10gs\x16\x01',
    ],
    (Ecu.fwdCamera, 0x733, None): [
      b'\x10y\x00 \x01',
    ],
    (Ecu.fwdRadar, 0x734, None): [
      b'\x10y\x000\x01',
    ],
  },
  CAR.MG_HS: {
    (Ecu.eps, 0x721, None): [
      b'\x10D\x860\x01',
      b'\x10\x92\x33\x87\x01',
    ],
    (Ecu.fwdCamera, 0x733, None): [
      b'\x10cT\x97\x01',
      b'\x10\x79\x07\x92\x01',
    ],
    (Ecu.fwdRadar, 0x734, None): [
      b'\x10cT\x99\x01',
      b'\x10\x92\x33\x80\x01',
    ],
  },
  # from the mg-zs branch
  CAR.MG_ZS: {
    (Ecu.eps, 0x721, None): [
      b'\x11\x06c\x94\x01',
    ],
    (Ecu.fwdCamera, 0x733, None): [
      b'\x10\x89wQ\x01',
    ],
    (Ecu.fwdRadar, 0x732, None): [
      b'\x11RuP\x01',
    ],
  },
}
