#!/usr/bin/env python3
"""
Panda-side fix-up of the head unit's ADASIS META (ICC_0x362, bus 0 -> ADAS).

The ADAS takes the unit of the speed-limit signs it reads from ICC_MetaSpdUnits; UK head units
send the message blank (km/h, country 0, traffic on the right), so a 30 mph sign set ACC to
30 km/h. fisker_fwd_modify rewrites the unit, traffic side and country per CP_SP.safetyParam.
"""

import unittest
from opendbc.car.structs import CarParams
from opendbc.sunnypilot.car.fisker.values_ext import FiskerSafetyFlagsSP
from opendbc.safety.tests.libsafety import libsafety_py

BLANK_META = bytes.fromhex("dc00299380003f00")   # as logged on a UK car, 2026-10-28
UK = 826


def _param(unit=0, side=0, country=0):
  return (unit << FiskerSafetyFlagsSP.META_UNIT_SHIFT) | (side << FiskerSafetyFlagsSP.META_SIDE_SHIFT) | \
         (country << FiskerSafetyFlagsSP.META_COUNTRY_SHIFT)


def _decode(d):
  return {"unit": (d[7] >> 4) & 1, "side": d[4] >> 7, "country": ((d[0] & 0x03) << 8) | d[1]}


class TestFiskerAdasisMeta(unittest.TestCase):
  def _init(self, param_sp):
    self.safety = libsafety_py.libsafety
    self.safety.set_current_safety_param_sp(param_sp)
    self.safety.set_safety_hooks(CarParams.SafetyModel.fisker, 0)
    self.safety.init_tests()

  def _dash(self, mph):
    d = bytearray(8)
    d[6] = 1 if mph else 0
    pkt = libsafety_py.make_CANPacket(0x531, 0, bytes(d))
    self.assertTrue(self.safety.safety_fwd_modify_hook(pkt))
    self.assertEqual(bytes(pkt.data[0:8]), bytes(d), "0x531 is only read, never changed")

  def _meta(self, d=BLANK_META, bus=0):
    pkt = libsafety_py.make_CANPacket(0x362, bus, d)
    self.assertTrue(self.safety.safety_fwd_modify_hook(pkt))
    return bytes(pkt.data[0:8])

  def _assert_only_fields_changed(self, before, after):
    mask = bytes([0xFC, 0x00, 0xFF, 0xFF, 0x7F, 0xFF, 0xFF, 0xEF])
    self.assertEqual(bytes(a & m for a, m in zip(after, mask, strict=True)), bytes(b & m for b, m in zip(before, mask, strict=True)))

  def test_follow_dash_mph(self):
    self._init(_param())
    self.assertEqual(self._meta(), BLANK_META, "unit untouched until the dash unit is known")
    self._dash(mph=True)
    out = self._meta()
    self.assertEqual(_decode(out), {"unit": 1, "side": 1, "country": 0})
    self._assert_only_fields_changed(BLANK_META, out)

  def test_follow_dash_kmh(self):
    self._init(_param())
    self._dash(mph=False)
    mph_meta = bytes(BLANK_META[:7]) + bytes([BLANK_META[7] | 0x10])
    self.assertEqual(_decode(self._meta(mph_meta))["unit"], 0)

  def test_forced_unit_ignores_dash(self):
    self._init(_param(unit=1))
    self.assertEqual(_decode(self._meta())["unit"], 1)
    self._dash(mph=False)
    self.assertEqual(_decode(self._meta())["unit"], 1)
    self._init(_param(unit=2))
    self._dash(mph=True)
    self.assertEqual(_decode(self._meta(bytes(BLANK_META[:7]) + b"\x10"))["unit"], 0)

  def test_passthrough(self):
    self._init(_param(unit=3))
    self._dash(mph=True)
    self.assertEqual(self._meta(), BLANK_META)

  def test_uk(self):
    self._init(_param(side=1, country=UK))
    self._dash(mph=True)
    out = self._meta()
    self.assertEqual(_decode(out), {"unit": 1, "side": 0, "country": UK})
    self._assert_only_fields_changed(BLANK_META, out)

  def test_right_and_max_country(self):
    self._init(_param(side=2, country=999))
    out = self._meta(bytes.fromhex("dfff299300003f00"))
    self.assertEqual(_decode(out)["side"], 1)
    self.assertEqual(_decode(out)["country"], 999)

  def test_other_bus_untouched(self):
    self._init(_param(unit=1, side=1, country=UK))
    self.assertEqual(self._meta(bus=2), BLANK_META)


if __name__ == "__main__":
  unittest.main()
