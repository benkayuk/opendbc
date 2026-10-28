import unittest

from opendbc.car.fisker.carcontroller import (LAT_CANDIDATE_WINDOW, LAT_NEXT_WINDOW_COUNT, LAT_NEXT_WINDOW_FROM,
                                              LAT_WINDOW_ENDS, lat_candidates, oem_reset_value)


def oem_stream(start_full, start_alive, reset, end, n):
  """OEM's 0x1D0 (secoc counter, alive, reset) for n frames, restarting at 1 with reset + 1 after `end`."""
  frames = []
  full, alive = start_full, start_alive
  for _ in range(n):
    frames.append((full, alive, reset))
    alive = (alive + 1) % 15
    if full == end:
      full, reset = 1, reset + 1
    else:
      full += 1
  return frames


class TestOemResetValue(unittest.TestCase):
  def test_gw_matches_oem(self):
    assert oem_reset_value(0x1235, 0x1) == 0x1235

  def test_gw_ahead_of_oem(self):
    # GW_Syn_All steps ~10 ms before OEM restarts its counter
    assert oem_reset_value(0x1236, 0x1) == 0x1235


class TestLatCandidates(unittest.TestCase):
  def test_mid_window_only_current(self):
    cands = lat_candidates(40, 3, 7)
    assert len(cands) == LAT_CANDIDATE_WINDOW
    assert all(reset == 7 for _, _, reset in cands)
    assert (40, 3, 7) in cands and (41, 4, 7) in cands

  def test_next_window_variants_near_end(self):
    cands = lat_candidates(LAT_NEXT_WINDOW_FROM, 0, 7)
    nxt = [c for c in cands if c[2] == 8]
    assert len(nxt) == len(LAT_WINDOW_ENDS) * LAT_NEXT_WINDOW_COUNT
    assert {ctr for ctr, _, _ in nxt} == set(range(1, LAT_NEXT_WINDOW_COUNT + 1))

  def test_skips_ends_already_passed(self):
    cands = lat_candidates(101, 0, 7)
    assert len([c for c in cands if c[2] == 8]) == 2 * LAT_NEXT_WINDOW_COUNT

  def test_covers_reset_with_host_lag(self):
    # Drive 2026-10-28: carstate saw OEM's restart up to 5 frames late and the EPS aborted.
    # Every OEM frame across the reset must be covered by a candidate the host stamped
    # from what it had seen `lag` frames earlier, whichever counter the window ends on.
    for end in LAT_WINDOW_ENDS:
      for lag in range(1, 6):
        stream = oem_stream(85, 9, 7, end, 40)
        for i in range(lag, len(stream)):
          assert stream[i] in lat_candidates(*stream[i - lag]), (end, lag, stream[i])
