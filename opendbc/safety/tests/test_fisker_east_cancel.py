#!/usr/bin/env python3
"""
Panda-side East-as-cancel (MFS_0x514, bus 0 -> ADAS).

The car ignores MFS_RiBtnEast while ACC is engaged, and North cancels ACC. A press of East that
starts while cruise is engaged is forwarded to the ADAS as a North press for as long as it is held.
"""

import unittest
from opendbc.car.structs import CarParams
from opendbc.car.fisker.fiskercan import fisker_plain_checksum
from opendbc.safety.tests.libsafety import libsafety_py

ACC_STANDBY, ACC_ACTIVE = 2, 3


def _0x514(east=0, north=0, south=0, alive=3):
  d = bytearray(8)
  d[1] = (alive & 0x0F) << 4
  d[2] = 0x11                                    # left-side buttons: untouched
  d[3] = (north & 0x03) << 5
  d[4] = ((east & 0x03) << 4) | (south & 0x03)
  d[6] = 0x40
  d[0] = fisker_plain_checksum(0x514, bytes(d))
  return bytes(d)


def _buttons(d):
  return {"east": (d[4] >> 4) & 3, "north": (d[3] >> 5) & 3, "south": d[4] & 3}


class TestFiskerEastCancel(unittest.TestCase):
  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.fisker, 0)
    self.safety.init_tests()

  def _acc(self, state):
    d = bytearray(8)
    d[4] = state & 0x0F
    self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x313, 2, bytes(d)))

  def _fwd(self, d, bus=0):
    pkt = libsafety_py.make_CANPacket(0x514, bus, d)
    self.assertTrue(self.safety.safety_fwd_modify_hook(pkt))
    out = bytes(pkt.data[0:8])
    self.assertEqual(out[0], fisker_plain_checksum(0x514, out), "checksum must stay valid")
    return out

  def test_east_engages_from_standby_untouched(self):
    self._acc(ACC_STANDBY)
    for east in (0, 1, 1, 2, 0):
      d = _0x514(east=east)
      self.assertEqual(self._fwd(d), d)

  def test_east_while_engaged_is_north(self):
    self._acc(ACC_ACTIVE)
    self.assertEqual(self._fwd(_0x514()), _0x514())
    for east in (1, 1, 2):
      out = self._fwd(_0x514(east=east, south=1))
      self.assertEqual(_buttons(out), {"east": 0, "north": 1, "south": 1})
      self.assertEqual(out[1:3] + out[5:8], _0x514()[1:3] + _0x514()[5:8], "other bytes untouched")
    # ACC drops to Standby mid-press: the rest of the same press is still North
    self._acc(ACC_STANDBY)
    self.assertEqual(_buttons(self._fwd(_0x514(east=1))), {"east": 0, "north": 1, "south": 0})
    # released, then a fresh press from Standby engages as normal
    self.assertEqual(self._fwd(_0x514()), _0x514())
    self.assertEqual(self._fwd(_0x514(east=1)), _0x514(east=1))

  def test_press_started_in_standby_stays_east(self):
    self._acc(ACC_STANDBY)
    self.assertEqual(self._fwd(_0x514(east=1)), _0x514(east=1))
    self._acc(ACC_ACTIVE)   # ACC engages from this very press
    self.assertEqual(self._fwd(_0x514(east=1)), _0x514(east=1))
    self.assertEqual(self._fwd(_0x514()), _0x514())

  def test_north_untouched(self):
    self._acc(ACC_ACTIVE)
    for north in (1, 2, 0):
      d = _0x514(north=north)
      self.assertEqual(self._fwd(d), d)

  def test_other_bus_untouched(self):
    self._acc(ACC_ACTIVE)
    d = _0x514(east=1)
    self.assertEqual(self._fwd(d, bus=2), d)


if __name__ == "__main__":
  unittest.main()
