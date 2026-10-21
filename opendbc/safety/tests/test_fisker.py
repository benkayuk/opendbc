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


class TestFiskerCanIgnition(unittest.TestCase):
  """The panda's voltage ignition line is never used on the Ocean (it reads high permanently),
  so CAN ignition from BCM_0x333.BCM_PwrMod (bus 0) is the only thing that starts openpilot."""

  PWR_MOD_RUN = 4

  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.safety.init_tests()

  def _msg(self, pwr_mod, bus=0, other_bits=0x00, length=8):
    dat = bytearray(length)
    dat[2] = other_bits | ((pwr_mod & 0x7) << 3)  # 3-bit @ Motorola start 21 -> data[2] bits 5..3
    return common.make_msg(bus, 0x333, dat=bytes(dat))

  def test_ignition_on(self):
    for pwr_mod in range(8):
      with self.subTest(pwr_mod=pwr_mod):
        self.safety.set_ignition_can(False)
        self.safety.ignition_can_hook(self._msg(pwr_mod))
        self.assertEqual(self.safety.get_ignition_can(), pwr_mod == self.PWR_MOD_RUN)

  def test_ignition_off(self):
    for pwr_mod in set(range(8)) - {self.PWR_MOD_RUN}:
      with self.subTest(pwr_mod=pwr_mod):
        self.safety.ignition_can_hook(self._msg(self.PWR_MOD_RUN))
        self.assertTrue(self.safety.get_ignition_can())
        self.safety.ignition_can_hook(self._msg(pwr_mod))
        self.assertFalse(self.safety.get_ignition_can())

  def test_neighboring_signals_ignored(self):
    # BCM_PwrModVld (data[2] bits 7..6) and BCM_IG1Sts (bits 2..1) share the byte
    self.safety.ignition_can_hook(self._msg(self.PWR_MOD_RUN, other_bits=0xC7))
    self.assertTrue(self.safety.get_ignition_can())
    self.safety.ignition_can_hook(self._msg(3, other_bits=0xC7))
    self.assertFalse(self.safety.get_ignition_can())

  def test_other_buses_and_lengths_ignored(self):
    for bus, length in ((1, 8), (2, 8), (0, 16)):
      with self.subTest(bus=bus, length=length):
        self.safety.set_ignition_can(False)
        self.safety.ignition_can_hook(self._msg(self.PWR_MOD_RUN, bus=bus, length=length))
        self.assertFalse(self.safety.get_ignition_can())


if __name__ == "__main__":
  unittest.main()
