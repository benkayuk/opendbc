"""
Our 0x121 accel frame must match the stock ADAS module's byte for byte while openpilot is the only
sender of it. The golden bytes are the stock module's with ACC Active (0x313 state 3) in drives
000000af--28a9f95752--1 and 000000b4--d0f733ebb2--5 (checksum and alive counter masked; they differ
per frame; the SecOC tail is stamped separately). openpilot never replaces the stock 0x117/0x118.
"""
from opendbc.can import CANPacker
from opendbc.car import Bus
from opendbc.car.fisker.fiskercan import FiskerCAN, fisker_plain_checksum
from opendbc.car.fisker.values import CAR, CANBUS, DBC, RELAY_CTRL_ADDR

STOCK_ACCEL_0X121 = bytes.fromhex("00d08005")            # bytes 0-3 with zero accel (raw 0x8005)


def make():
  return FiskerCAN(None, CANPacker(DBC[CAR.FISKER_OCEAN][Bus.pt]))


def mask(data: bytes) -> bytes:
  m = bytearray(data)
  m[0] = 0
  m[1] &= 0xF0   # AliveCounter is the low nibble of byte 1
  return bytes(m)


def test_0x121_matches_stock_idle_and_scales():
  _, data, _ = make().create_accel_command(0.0, 7)
  assert mask(data)[:4] == STOCK_ACCEL_0X121
  assert data[1] & 0xF == 7
  for accel in (-3.5, -0.85, 0.5, 2.0):
    d = make().create_accel_command(accel, 0)[1]
    raw = (d[2] << 8) | d[3]
    assert abs(raw * 0.0004882 - 16 - accel) < 0.001


def test_0x121_valid_even_when_not_commanding():
  # the stock module sends AccelVld=Valid on every frame, with zero accel when it is not commanding
  for accel in (0.0, 1.0):
    assert (make().create_accel_command(accel, 0)[1][1] >> 4) & 0x3 == 1


def test_checksum_valid():
  addr, data, _ = make().create_accel_command(1.0, 3)
  assert fisker_plain_checksum(addr, bytes([0]) + data[1:]) == data[0]


def test_relay_release_frame():
  addr, data, bus = make().create_relay_release(0x07)
  assert addr == RELAY_CTRL_ADDR and bus == CANBUS.pt and len(data) == 8 and data[0] == 0x07


def test_accel_cap_matches_panda():
  from opendbc.car.fisker.interface import CarInterface
  from opendbc.car.fisker.values import CarControllerParams
  assert CarControllerParams.ACCEL_MAX == 1.0
  assert CarInterface.get_pid_accel_limits(None, None, 10.0, 20.0) == (CarControllerParams.ACCEL_MIN, 1.0)
  d = make().create_accel_command(CarControllerParams.ACCEL_MAX, 0)[1]
  assert (d[2] << 8) | d[3] == 34822   # FISKER_ACCEL_MAX in safety/modes/fisker.h
