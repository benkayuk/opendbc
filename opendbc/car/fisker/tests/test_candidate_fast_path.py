"""
The 0x1D0 candidate window's fast paths must give exactly the bytes of the slow ones:
the cached single-block CMAC vs pycryptodome's CMAC, and patching alive into one packed
frame vs packing each candidate.
"""

import random
import unittest

from Crypto.Cipher import AES
from Crypto.Hash import CMAC

from opendbc.can import CANPacker
from opendbc.car.fisker.fiskercan import FiskerCAN
from opendbc.car.fisker.secoc import _cmac24


class TestCandidateFastPath(unittest.TestCase):
  def test_cmac_matches_pycryptodome(self):
    rng = random.Random(0)
    keys = [rng.randbytes(16) for _ in range(20)]
    for i in range(2000):
      key = keys[i % 20]
      data = rng.randbytes(rng.randint(0, 24))
      ref = CMAC.new(key, ciphermod=AES)
      ref.update(data)
      self.assertEqual(_cmac24(key, data), ref.digest()[:3], (key.hex(), data.hex()))

  def test_alive_patch_matches_packing(self):
    rng = random.Random(0)
    fcan = FiskerCAN(None, CANPacker("fisker_ocean_adas"))
    for _ in range(2000):
      angle = rng.uniform(-780, 780)
      base = fcan.create_steering_control(angle, 0)
      for alive in (0, 7, 14, 15, rng.randint(0, 31)):
        self.assertEqual(fcan.steering_control_with_alive(base, alive), fcan.create_steering_control(angle, alive))


if __name__ == "__main__":
  unittest.main()
