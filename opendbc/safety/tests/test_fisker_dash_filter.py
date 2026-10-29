#!/usr/bin/env python3
"""
Panda-side filter for the ADAS's spurious ACC-fault warnings (ADAS_0x317, bus 2 -> cluster).

Every ACC switch-off passes through ADAS_Sts_ACC_ICC 10 for 20-40 ms, and ACC sits in 9/10
while the car is out of a forward gear; each time the ADAS lights the yellow ADAS fault
telltale and/or shows "ACC fault" with the fault chime. fisker_fwd_modify clears just those
values while the episode is hideable, and leaves a failure that persists in D, a critical
fault and every other warning alone.
"""

import unittest
from opendbc.car.structs import CarParams
from opendbc.car.fisker.fiskercan import fisker_plain_checksum
from opendbc.safety.tests.libsafety import libsafety_py

GEAR_P, GEAR_R, GEAR_N, GEAR_D = 1, 3, 2, 4
ACC_TEXT, OTHER_TEXT = 7, 6     # ADAS_IDS_WarnTxtReq: ACC_fault, Lane_keeping_system_fault
FAULT_CHIME, DISENGAGE_CHIME = 3, 2


def _0x317(text=0, telltale=0, chime=0, alive=5):
  d = bytearray(8)
  d[1] = alive & 0x0F
  d[2] = (chime & 0x1F) << 3
  d[3] = text & 0x3F
  d[5] = (telltale & 0x03) << 6
  d[6] = 0x8F  # ADAS_SysFltTyp & co: untouched
  d[0] = fisker_plain_checksum(0x317, bytes(d))
  return d


class TestFiskerDashFilter(unittest.TestCase):
  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.fisker, 0)
    self.safety.init_tests()
    self.t = 1_000_000
    self.safety.set_timer(self.t)

  def _advance(self, us):
    self.t += us
    self.safety.set_timer(self.t)

  def _acc(self, state):
    d = bytearray(8)
    d[4] = state & 0x0F
    self.safety.safety_rx_hook(libsafety_py.make_CANPacket(0x313, 2, bytes(d)))

  def _gear(self, gear):
    d = bytearray(16)
    d[3] = gear & 0x0F
    pkt = libsafety_py.make_CANPacket(0x214, 0, bytes(d))
    self.assertTrue(self.safety.safety_fwd_modify_hook(pkt))
    self.assertEqual(bytes(pkt.data[0:16]), bytes(d), "0x214 is only read, never changed")

  def _warn(self, **kw):
    """Forward one 0x317 and return (text, telltale, chime) as the cluster gets it."""
    d = _0x317(**kw)
    pkt = libsafety_py.make_CANPacket(0x317, 2, bytes(d))
    self.assertTrue(self.safety.safety_fwd_modify_hook(pkt))
    out = bytes(pkt.data[0:8])
    self.assertEqual(out[0], fisker_plain_checksum(0x317, out), "CheckSum must match what's sent")
    self.assertEqual(out[1], d[1], "AliveCounter untouched")
    self.assertEqual(out[6], d[6])
    return out[3] & 0x3F, out[5] >> 6, out[2] >> 3

  def _cancel(self):
    # North while active: 3 -> 8 -> 10 for ~40 ms -> 2
    for state in (3, 8):
      self._acc(state)
      self._advance(20_000)
    self._acc(10)
    self._advance(40_000)
    self._acc(2)

  def test_untouched_without_acc_failure(self):
    self._gear(GEAR_D)
    self._acc(3)
    self.assertEqual(self._warn(text=ACC_TEXT, telltale=1, chime=FAULT_CHIME), (ACC_TEXT, 1, FAULT_CHIME))
    self.assertEqual(self._warn(telltale=2), (0, 2, 0))

  def test_cancel_blip_hidden(self):
    self._gear(GEAR_D)
    self._cancel()
    self._advance(20_000)
    self.assertEqual(self._warn(text=ACC_TEXT, telltale=1, chime=FAULT_CHIME), (0, 0, 0))

  def test_delayed_text_hidden_until_it_clears(self):
    self._gear(GEAR_D)
    self._cancel()
    # the telltale only trails the blip by ~50 ms; later it goes through again
    self._advance(2_000_000)
    self.assertEqual(self._warn(telltale=1), (0, 1, 0))
    # "ACC fault" text 6 s after the cancel, held ~5 s: hidden for all of it
    self._advance(4_000_000)
    for _ in range(50):
      self.assertEqual(self._warn(text=ACC_TEXT, chime=FAULT_CHIME), (0, 0, 0))
      self._advance(100_000)
    self.assertEqual(self._warn(), (0, 0, 0))
    # a new ACC-fault text long after the window shows
    self._advance(1_000_000)
    self.assertEqual(self._warn(text=ACC_TEXT, chime=FAULT_CHIME), (ACC_TEXT, 0, FAULT_CHIME))

  def test_other_warnings_untouched(self):
    self._gear(GEAR_D)
    self._cancel()
    self.assertEqual(self._warn(text=OTHER_TEXT, chime=FAULT_CHIME), (OTHER_TEXT, 0, FAULT_CHIME))
    self.assertEqual(self._warn(telltale=2, chime=FAULT_CHIME), (0, 2, FAULT_CHIME))
    # the disengagement chime isn't a fault chime
    self.assertEqual(self._warn(text=ACC_TEXT, chime=DISENGAGE_CHIME), (0, 0, DISENGAGE_CHIME))

  def test_persistent_failure_in_drive_shows(self):
    self._gear(GEAR_D)
    self._acc(3)
    self._acc(10)
    self.assertEqual(self._warn(text=ACC_TEXT, telltale=1), (0, 0, 0))
    for _ in range(11):
      self._advance(100_000)
      self._acc(10)
    self.assertEqual(self._warn(text=ACC_TEXT, telltale=1, chime=FAULT_CHIME), (ACC_TEXT, 1, FAULT_CHIME))
    # ACC recovering doesn't make the same fault hideable again
    self._acc(2)
    self.assertEqual(self._warn(text=ACC_TEXT, telltale=1), (ACC_TEXT, 1, 0))

  def test_out_of_forward_gear_hidden(self):
    for gear in (GEAR_R, GEAR_P, GEAR_N):
      with self.subTest(gear=gear):
        self.setUp()
        self._gear(GEAR_D)
        self._acc(2)
        self._gear(gear)
        for _ in range(30):
          self._acc(10)
          self._advance(200_000)
          self._gear(gear)
          self.assertEqual(self._warn(text=ACC_TEXT, telltale=1, chime=FAULT_CHIME), (0, 0, 0))
        # back in D: ACC stays 10 for ~90 ms before Off -> Init -> Standby
        self._gear(GEAR_D)
        self._acc(10)
        self._advance(90_000)
        self._acc(0)
        self.assertEqual(self._warn(telltale=1), (0, 0, 0))

  def test_reset_on_init(self):
    self._gear(GEAR_R)
    self._acc(10)
    self.setUp()
    self.assertEqual(self._warn(text=ACC_TEXT, telltale=1), (ACC_TEXT, 1, 0))

  def test_vehicle_side_0x317_ignored(self):
    self._gear(GEAR_D)
    self._cancel()
    d = _0x317(text=ACC_TEXT, telltale=1)
    pkt = libsafety_py.make_CANPacket(0x317, 0, bytes(d))
    self.assertTrue(self.safety.safety_fwd_modify_hook(pkt))
    self.assertEqual(bytes(pkt.data[0:8]), bytes(d))


if __name__ == "__main__":
  unittest.main()
