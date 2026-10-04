#!/usr/bin/env python3
import unittest

from opendbc.car.structs import CarParams
import opendbc.safety.tests.common as common
from opendbc.safety.tests.libsafety import libsafety_py

ICC_SETTINGS = 0x52A
ICC_FRAME = bytes.fromhex("BE2026A043008220")
TIMEOUT_US = 500_000


class TestFiskerIccSettings(unittest.TestCase):
  """ICC_0x52A: openpilot re-sends the ICC's frame on bus 2; panda drops the ICC's original
  (bus 0 -> 2) only while openpilot's copies keep arriving."""

  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.safety.set_timer(0)  # init_tests() also resets the timer to 0; keep init's timestamp consistent
    self.safety.set_safety_hooks(CarParams.SafetyModel.fisker, 0)
    self.safety.init_tests()

  def _tx(self, bus):
    return self.safety.safety_tx_hook(common.make_msg(bus, ICC_SETTINGS, dat=ICC_FRAME))

  def _fwd(self, bus):
    return self.safety.safety_fwd_hook(bus, ICC_SETTINGS)

  def test_tx_allowed_on_adas_bus_only(self):
    for controls_allowed in (False, True):
      self.safety.set_controls_allowed(controls_allowed)
      self.assertTrue(self._tx(2))
      self.assertFalse(self._tx(0))
      self.assertFalse(self._tx(1))

  def test_tx_allowed_with_long_flag(self):
    self.safety.set_safety_hooks(CarParams.SafetyModel.fisker, 1)
    self.assertTrue(self._tx(2))

  def test_blocked_at_init(self):
    # openpilot is already sending when the safety mode starts: block from the first ICC frame
    self.assertEqual(self._fwd(0), -1)
    self.safety.set_timer(TIMEOUT_US - 1)
    self.assertEqual(self._fwd(0), -1)

  def test_forwarded_if_openpilot_never_sends(self):
    self.safety.set_timer(TIMEOUT_US)
    self.assertEqual(self._fwd(0), 2)

  def test_blocked_while_openpilot_sends(self):
    # each of our frames extends the window past the init window
    for t in range(0, 5_000_000, 200_000):
      self.safety.set_timer(t)
      self.assertTrue(self._tx(2))
      self.assertEqual(self._fwd(0), -1)
    self.safety.set_timer(t + TIMEOUT_US - 1)
    self.assertEqual(self._fwd(0), -1)
    self.safety.set_timer(t + TIMEOUT_US)
    self.assertEqual(self._fwd(0), 2)

  def test_fallback_to_icc_when_openpilot_stops(self):
    self.assertTrue(self._tx(2))
    self.safety.set_timer(TIMEOUT_US)
    self.assertEqual(self._fwd(0), 2)

    # resumes blocking as soon as openpilot sends again
    self.assertTrue(self._tx(2))
    self.assertEqual(self._fwd(0), -1)

  def test_window_restarts_on_safety_mode_init(self):
    self.safety.set_timer(2 * TIMEOUT_US)
    self.assertEqual(self._fwd(0), 2)
    self.safety.set_safety_hooks(CarParams.SafetyModel.fisker, 0)
    self.assertEqual(self._fwd(0), -1)

  def test_adas_side_not_blocked(self):
    # only the ICC -> ADAS direction is intercepted
    self.assertTrue(self._tx(2))
    self.assertEqual(self._fwd(2), 0)

  def test_no_relay_malfunction(self):
    # 0x52A on bus 2 must not trip relay malfunction (that would stop all forwarding)
    self.safety.safety_rx_hook(common.make_msg(2, ICC_SETTINGS, dat=ICC_FRAME))
    self.assertFalse(self.safety.get_relay_malfunction())


class TestFiskerIcc0x35B(unittest.TestCase):
  """ICC_0x35B: optional relay — the ICC's original is only blocked once openpilot's copies are
  actually arriving on bus 2, and forwarding resumes if they stop."""

  ADDR = 0x35B

  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.safety.set_timer(0)  # init_tests() also resets the timer to 0; keep init's timestamp consistent
    self.safety.set_safety_hooks(CarParams.SafetyModel.fisker, 0)
    self.safety.init_tests()

  def _tx(self, bus):
    return self.safety.safety_tx_hook(common.make_msg(bus, self.ADDR, 8))

  def _fwd(self, bus):
    return self.safety.safety_fwd_hook(bus, self.ADDR)

  def test_tx_allowed_on_adas_bus_only(self):
    for param in (0, 1):
      self.safety.set_safety_hooks(CarParams.SafetyModel.fisker, param)
      self.assertTrue(self._tx(2))
      self.assertFalse(self._tx(0))
      self.assertFalse(self._tx(1))

  def test_forwarded_while_openpilot_not_relaying(self):
    # no overrides configured -> openpilot never sends 0x35B -> ICC's frame always forwarded
    for t in (0, TIMEOUT_US, 10 * TIMEOUT_US):
      self.safety.set_timer(t)
      self.assertEqual(self._fwd(0), 2)

  def test_blocked_while_openpilot_relays(self):
    for t in range(0, 5_000_000, 200_000):
      self.safety.set_timer(t)
      self.assertTrue(self._tx(2))
      self.assertEqual(self._fwd(0), -1)
    self.safety.set_timer(t + TIMEOUT_US - 1)
    self.assertEqual(self._fwd(0), -1)
    self.safety.set_timer(t + TIMEOUT_US)
    self.assertEqual(self._fwd(0), 2)

  def test_reset_on_safety_mode_init(self):
    self.assertTrue(self._tx(2))
    self.assertEqual(self._fwd(0), -1)
    self.safety.set_safety_hooks(CarParams.SafetyModel.fisker, 0)
    self.assertEqual(self._fwd(0), 2)

  def test_independent_of_0x52a(self):
    # relaying 0x52A doesn't block 0x35B, and vice versa
    self.assertTrue(self.safety.safety_tx_hook(common.make_msg(2, ICC_SETTINGS, dat=ICC_FRAME)))
    self.assertEqual(self._fwd(0), 2)
    self.safety.set_timer(TIMEOUT_US)
    self.assertTrue(self._tx(2))
    self.assertEqual(self._fwd(0), -1)
    self.assertEqual(self.safety.safety_fwd_hook(0, ICC_SETTINGS), 2)

  def test_adas_side_not_blocked(self):
    self.assertTrue(self._tx(2))
    self.assertEqual(self._fwd(2), 0)


class TestFiskerCruiseState(unittest.TestCase):
  """Cruise engagement follows the ADAS module's ACC state (ADAS_0x313, bus 2)."""

  # ADAS_Sts_ACC_ICC: Active, Override, Standstill_active, Standstill_wait, Standstill_GoNotification
  ENGAGED_STATES = {3, 4, 5, 6, 11}

  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.fisker, 0)
    self.safety.init_tests()

  def _state_msg(self, addr, bus, acc_state):
    dat = bytearray(8)
    dat[4] = acc_state & 0x0F  # 4-bit @ Motorola start 35 -> data[4] low nibble
    return common.make_msg(bus, addr, dat=bytes(dat))

  def _rx_acc_state(self, acc_state, addr=0x313, bus=2):
    self.safety.safety_rx_hook(self._state_msg(addr, bus, acc_state))

  def test_engaged_states(self):
    for acc_state in range(16):
      with self.subTest(acc_state=acc_state):
        self._rx_acc_state(0)
        self.assertFalse(self.safety.get_controls_allowed())
        self._rx_acc_state(acc_state)
        self.assertEqual(self.safety.get_controls_allowed(), acc_state in self.ENGAGED_STATES)

  def test_disengage_on_acc_exit(self):
    for acc_state in set(range(16)) - self.ENGAGED_STATES:
      with self.subTest(acc_state=acc_state):
        self._rx_acc_state(0)
        self._rx_acc_state(3)
        self.assertTrue(self.safety.get_controls_allowed())
        self._rx_acc_state(acc_state)
        self.assertFalse(self.safety.get_controls_allowed())

  def test_stays_engaged_through_standstill(self):
    self._rx_acc_state(0)
    for acc_state in (3, 5, 6, 11, 3, 4):
      self._rx_acc_state(acc_state)
      self.assertTrue(self.safety.get_controls_allowed())

  def test_other_sources_ignored(self):
    # VCU basic cruise (0x358) no longer engages; 0x313 only counts from the ADAS side (bus 2)
    for addr, bus in ((0x358, 0), (0x313, 0)):
      with self.subTest(addr=hex(addr), bus=bus):
        self._rx_acc_state(0)
        self._rx_acc_state(3, addr=addr, bus=bus)
        self.assertFalse(self.safety.get_controls_allowed())


class TestFiskerRelay(unittest.TestCase):
  """The relay arbiter for 0x1D0 / 0x1C0 / 0x121 (see fisker.h). Panda decides forwarding before it
  runs the RX hook for the same frame, so stock() does the same: fwd first, then rx."""

  STEER, LAT, ACCEL = 0x1D0, 0x1C0, 0x121
  CTRL = 0x5FE
  FAIL_US = 50_000

  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.safety.set_timer(0)
    self.safety.set_safety_hooks(CarParams.SafetyModel.fisker, 1)   # FiskerSafetyFlags.LONG_CONTROL
    self.safety.init_tests()
    self.t = 0
    self.safety.set_timer(self.t)

  def advance(self, us):
    self.t += us
    self.safety.set_timer(self.t)

  @staticmethod
  def _dat(addr, ctr):
    d = bytearray(8)
    d[1] = ctr & 0x0F
    if addr == 0x1D0:
      d[2], d[3] = 0x30, 0xC0                # 0 deg
    elif addr == 0x121:
      d[2], d[3] = 0x80, 0x05                # inactive accel
    return bytes(d)

  def stock(self, addr, ctr):
    """A stock frame on bus 2: returns True if panda forwards it."""
    fwd = self.safety.safety_fwd_hook(2, addr)
    self.safety.safety_rx_hook(common.make_msg(2, addr, dat=self._dat(addr, ctr)))
    return fwd != -1

  def ours(self, addr, ctr):
    return bool(self.safety.safety_tx_hook(common.make_msg(0, addr, dat=self._dat(addr, ctr))))

  def release(self, mask):
    return bool(self.safety.safety_tx_hook(common.make_msg(0, self.CTRL, dat=bytes([mask]) + bytes(7))))

  def acc_state(self, state):
    dat = bytearray(8)
    dat[4] = state & 0x0F
    self.safety.safety_rx_hook(common.make_msg(2, 0x313, dat=bytes(dat)))

  def engage(self, addr=STEER, stock_ctr=5, ours_ctr=6):
    """Stock stream up to stock_ctr forwarded, then our takeover frame."""
    self.safety.set_controls_allowed(True)
    self.safety.set_controls_allowed_lateral(True)
    for c in range(stock_ctr - 2, stock_ctr + 1):
      self.assertTrue(self.stock(addr, c % 15))
    self.assertTrue(self.ours(addr, ours_ctr % 15))

  # ---- takeover ----
  def test_stock_forwarded_until_we_send(self):
    for c in range(20):
      self.assertTrue(self.stock(self.STEER, c % 15))

  def test_takeover_blocks_stock(self):
    self.engage()
    self.assertFalse(self.stock(self.STEER, 6))
    self.assertTrue(self.ours(self.STEER, 7))
    self.assertFalse(self.stock(self.STEER, 7))

  def test_takeover_window(self):
    self.safety.set_controls_allowed(True)
    self.stock(self.STEER, 5)
    self.assertFalse(self.ours(self.STEER, 9))     # 4 ahead of the stock counter
    self.assertTrue(self.ours(self.STEER, 8))      # 3 ahead
    self.setUp()
    self.safety.set_controls_allowed(True)
    self.stock(self.STEER, 5)
    self.assertTrue(self.ours(self.STEER, 5))      # repeat of the last stock frame
    self.setUp()
    self.safety.set_controls_allowed(True)
    self.stock(self.STEER, 5)
    self.assertFalse(self.ours(self.STEER, 4))     # a step back
    self.assertFalse(self.ours(self.STEER, 14))

  def test_takeover_window_wraps(self):
    self.safety.set_controls_allowed(True)
    self.stock(self.STEER, 13)
    self.assertTrue(self.ours(self.STEER, 1))      # 13 -> 14, 0, 1 is 3 ahead

  def test_ours_strictly_forward(self):
    self.engage()                                  # ours at 6
    self.assertFalse(self.ours(self.STEER, 6))     # repeat
    self.assertFalse(self.ours(self.STEER, 5))     # back
    self.assertFalse(self.ours(self.STEER, 10))    # +4
    self.assertTrue(self.ours(self.STEER, 9))      # +3 (a dropped frame or two)
    self.assertTrue(self.ours(self.STEER, 10))

  def test_ours_rejected_when_not_allowed(self):
    self.stock(self.STEER, 5)
    self.assertFalse(self.ours(self.STEER, 6))     # neither cruise nor MADS: no takeover, stock keeps flowing
    self.assertTrue(self.stock(self.STEER, 6))

  def test_counter_15_rejected(self):
    self.safety.set_controls_allowed(True)
    self.stock(self.STEER, 5)
    self.assertFalse(self.ours(self.STEER, 15))

  # ---- release ----
  def test_release_frame_resumes_exactly_at_ours_plus_one(self):
    self.engage(stock_ctr=5, ours_ctr=6)           # lead 1
    self.assertFalse(self.stock(self.STEER, 6))
    self.assertTrue(self.ours(self.STEER, 7))
    self.assertTrue(self.ours(self.STEER, 8))
    self.assertFalse(self.stock(self.STEER, 7))
    self.assertFalse(self.release(0x01))           # consumed and rejected, never sent
    self.assertFalse(self.ours(self.STEER, 9))     # frozen
    self.assertFalse(self.stock(self.STEER, 8))    # stock 8 would repeat our last frame: still blocked
    self.assertTrue(self.stock(self.STEER, 9))     # continues ours (8)
    self.assertTrue(self.stock(self.STEER, 10))

  def test_release_when_stock_is_ahead_jumps_at_most_three(self):
    self.engage(stock_ctr=5, ours_ctr=6)           # ours at 6
    for c in (6, 7, 8):
      self.assertFalse(self.stock(self.STEER, c))  # stock runs ahead of us (we dropped frames)
    self.release(0x01)
    # stock counter 8, ours 6: next stock frame is 9, three past ours+1... d = 8 - 6 = 2 -> resume now
    self.assertTrue(self.stock(self.STEER, 9))

  def test_release_when_stock_is_far_ahead_still_bounded(self):
    self.engage(stock_ctr=5, ours_ctr=6)
    for c in range(6, 12):
      self.stock(self.STEER, c)
    self.release(0x01)
    # stock is 5 ahead of ours (d=5 > 3): wait for it to come around instead of jumping
    forwarded = [self.stock(self.STEER, c % 15) for c in range(12, 12 + 15)]
    self.assertFalse(forwarded[0])
    self.assertTrue(any(forwarded))

  def test_release_mask_selects_relays(self):
    for addr in (self.STEER, self.LAT):
      self.safety.set_controls_allowed(True)
      self.stock(addr, 5)
      self.assertTrue(self.ours(addr, 6))
    self.release(0x01)                             # steering only
    self.assertFalse(self.ours(self.STEER, 7))
    self.assertTrue(self.ours(self.LAT, 7))
    self.assertFalse(self.stock(self.LAT, 6))

  def test_release_when_not_taken_is_ignored(self):
    self.release(0x07)
    self.assertTrue(self.stock(self.STEER, 1))

  def test_release_then_retake(self):
    self.engage()                                  # ours at 6
    self.release(0x01)
    self.assertFalse(self.stock(self.STEER, 6))    # ours is already at 6: stock 6 would repeat it
    self.assertTrue(self.stock(self.STEER, 7))     # continues ours: resumes
    self.assertTrue(self.stock(self.STEER, 8))
    self.assertTrue(self.ours(self.STEER, 9))      # and we can take over again, continuing the stock counter
    self.assertFalse(self.stock(self.STEER, 9))

  # ---- fail-safe: the c0 defect (stale lateral flag kept the stock stream blocked for 2.4 s) ----
  def test_stale_lateral_flag_cannot_starve_the_vehicle_bus(self):
    self.engage()
    self.assertTrue(self.safety.get_controls_allowed_lateral())   # flag still set, but we stopped sending
    blocked = 0
    for i in range(30):
      self.advance(10_000)
      if not self.stock(self.STEER, (7 + i) % 15):
        blocked += 1
    self.assertLessEqual(blocked, 6)                              # ~50 ms, not seconds
    self.assertTrue(self.stock(self.STEER, 7 + 30))

  def test_failsafe_returns_stock_and_allows_retake(self):
    self.engage()
    self.advance(self.FAIL_US + 5_000)
    self.assertTrue(self.stock(self.STEER, 6))
    self.assertTrue(self.stock(self.STEER, 7))

  def test_steady_stream_never_leaks_stock(self):
    self.engage()
    for i in range(1, 200):
      self.advance(10_000)
      self.assertFalse(self.stock(self.STEER, (6 + i) % 15), i)
      self.assertTrue(self.ours(self.STEER, (7 + i) % 15), i)

  # ---- longitudinal (0x121) ----
  def test_accel_takeover_needs_cruise(self):
    self.stock(self.ACCEL, 5)
    self.assertFalse(self.ours(self.ACCEL, 6))     # inactive accel is allowed by the signal check, but cruise is off
    self.assertTrue(self.stock(self.ACCEL, 6))

  def test_accel_relay_follows_cruise_and_releases_on_drop(self):
    self.acc_state(2)
    self.acc_state(3)
    for c in (3, 4, 5):
      self.assertTrue(self.stock(self.ACCEL, c))
    self.assertTrue(self.ours(self.ACCEL, 6))
    self.assertFalse(self.stock(self.ACCEL, 6))
    self.acc_state(8)                              # ACC main pressed: Deactivation_other
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertFalse(self.ours(self.ACCEL, 7))     # frozen at once, no frame of ours after the drop
    self.assertTrue(self.stock(self.ACCEL, 7))     # stock stream resumes at exactly our last counter + 1
    self.assertTrue(self.stock(self.ACCEL, 8))

  def test_accel_capped_at_one(self):
    self.acc_state(3)
    self.stock(self.ACCEL, 5)
    for n, (hi, lo, ok) in enumerate(((0x88, 0x06, True), (0x88, 0x07, False), (0x90, 0x06, False), (0x64, 0x04, True))):
      d = bytearray(self._dat(self.ACCEL, 6 + n))
      d[2], d[3] = hi, lo                          # 0x8806 = 34822 = +1.0 m/s2, 0x8807 just above, 0x9006 = +2.0, 0x6404 = -3.5
      with self.subTest(raw=hex(hi << 8 | lo)):
        self.assertEqual(bool(self.safety.safety_tx_hook(common.make_msg(0, self.ACCEL, dat=bytes(d)))), ok)

  def test_accel_relay_releases_on_brake_path_too(self):
    self.acc_state(3)
    self.stock(self.ACCEL, 5)
    self.assertTrue(self.ours(self.ACCEL, 6))
    self.safety.set_controls_allowed(False)        # any other path that drops controls_allowed
    self.assertFalse(self.ours(self.ACCEL, 7))

  def test_lateral_unaffected_by_cruise_dropping(self):
    self.engage()
    self.acc_state(2)
    self.safety.set_controls_allowed_lateral(True)
    self.assertTrue(self.ours(self.STEER, 7))

  def test_accel_not_relayed_without_long_flag(self):
    self.safety.set_safety_hooks(CarParams.SafetyModel.fisker, 0)
    self.safety.init_tests()
    self.safety.set_controls_allowed(True)
    for c in range(10):
      self.assertTrue(self.stock(self.ACCEL, c))
    self.assertFalse(self.ours(self.ACCEL, 11))    # not whitelisted without the long flag

  # ---- scope: openpilot never replaces 0x117 / 0x118 ----
  def test_stock_117_118_always_forwarded_and_not_sendable(self):
    self.acc_state(2)
    self.acc_state(3)
    self.safety.set_controls_allowed_lateral(True)
    for addr in (0x117, 0x118):
      with self.subTest(addr=hex(addr)):
        self.assertEqual(self.safety.safety_fwd_hook(2, addr), 0)
        self.assertFalse(self.safety.safety_tx_hook(common.make_msg(0, addr, dat=bytes.fromhex("00004b4050a1ff28"))))


if __name__ == "__main__":
  unittest.main()
