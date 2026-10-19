import pytest

from opendbc.can import CANPacker
from opendbc.car import Bus, structs
from opendbc.car.fisker.carcontroller import CarController, apply_fisker_steer_angle_limits
from opendbc.car.fisker.carstate import CarState
from opendbc.car.fisker.interface import CarInterface
from opendbc.car.fisker.values import CANBUS, CAR, DBC
from opendbc.car.lateral import apply_std_steer_angle_limits
from opendbc.car.fisker.values import CarControllerParams


@pytest.fixture
def car():
  cp = CarInterface.get_non_essential_params(CAR.FISKER_OCEAN)
  cp.secOcKeyAvailable = True
  sp = CarInterface.get_non_essential_params_sp(cp, CAR.FISKER_OCEAN)
  cs = CarState(cp, sp)
  return cp, sp, cs


@pytest.mark.parametrize('last,measured,speed', [(-1.8125, 3.3125, 18.52), (18.875, 11.6875, 18.78), (46.875, 41.25, 10.4)])
@pytest.mark.parametrize('cruise', [False, True])
def test_override_releases_immediately_without_angle_jump(car, last, measured, speed, cruise):
  cp, sp, cs = car
  controller = CarController(DBC[CAR.FISKER_OCEAN], cp, sp)
  controller.secoc_key_verified = True
  controller.secoc_key = bytes(16)  # synthetic key, no vehicle credentials
  controller.apply_angle_last = last
  controller.was_lat_engaged_prev = True
  cs.out = structs.CarState(steeringAngleDeg=measured, vEgoRaw=speed)
  cs.out.cruiseState.enabled = cruise
  cs.driver_intervening = True
  cc = structs.CarControl(latActive=True)
  cc.actuators.steeringAngleDeg = last
  cc_sp = structs.CarControlSP()
  cc_sp.mads.active = True
  _, frames = controller.update(cc.as_reader(), cc_sp, cs, 0)
  messages = {addr: data for addr, data, bus in frames if bus == CANBUS.pt}
  assert (messages[0x1C0][3] >> 6) & 3 == 0
  assert messages[0x1C0][2] & 32  # override remains signalled
  sent_angle = int.from_bytes(messages[0x1D0][2:4], 'big') * .0625 - 780
  assert abs(sent_angle - last) < 2.0
  assert abs(sent_angle - measured) > 3


@pytest.mark.parametrize('speed', [0, 5, 10, 25, 40])
@pytest.mark.parametrize('last,target', [(-20, 20), (20, -20), (0, 900), (0, -900)])
def test_active_limits_unchanged(speed, last, target):
  assert apply_fisker_steer_angle_limits(target, last, speed, 0, True, True) == apply_std_steer_angle_limits(
    target, last, speed, 0, True, CarControllerParams.ANGLE_LIMITS)


def test_disabled_heartbeat_matches_measured_angle():
  assert apply_fisker_steer_angle_limits(50, 20, 15, -10, False, False) == -10


@pytest.mark.parametrize('cruise', [False, True])
def test_disengagement_fade_respects_remaining_permission(car, cruise):
  cp, sp, cs = car
  controller = CarController(DBC[CAR.FISKER_OCEAN], cp, sp)
  controller.secoc_key_verified = True
  controller.secoc_key = bytes(16)
  controller.apply_angle_last = 20
  controller.was_lat_engaged_prev = True
  controller.lat_fade_remaining = controller.FADE_TICKS
  cs.out = structs.CarState(steeringAngleDeg=-10, vEgoRaw=15)
  cs.out.cruiseState.enabled = cruise
  cc = structs.CarControl(latActive=False)
  _, frames = controller.update(cc.as_reader(), structs.CarControlSP(), cs, 0)
  messages = {addr: data for addr, data, bus in frames if bus == CANBUS.pt}
  assert (messages[0x1C0][3] >> 6) & 3 == 0
  angle = int.from_bytes(messages[0x1D0][2:4], 'big') * .0625 - 780
  if cruise:
    assert 18 < angle < 20  # ACC still leaves Panda's active rate check enabled
  else:
    assert angle == -10  # no permission: inactive check requires measured angle


def test_fault_requires_valid_eps_recovery(car):
  cp, sp, cs = car
  parsers = cs.get_can_parsers(cp, sp)
  packer = CANPacker(DBC[CAR.FISKER_OCEAN][Bus.pt])

  def update(state, validity, abort):
    frame = packer.make_can_msg('EPS_0x1C2', CANBUS.pt, {
      'EPS_AdasLatCtrlSts': state, 'EPS_AdasLatCtrlStsVld': validity, 'EPS_AbortFb': abort})
    for parser in parsers.values():
      parser.update([(0, [frame])])
    return cs.update(parsers)[0].steerFaultTemporary

  assert not update(0, 0, 0)  # startup is not a historical fault
  assert not update(2, 1, 0)
  assert not update(0, 0, 1)  # expected driver intervention alone
  assert update(3, 1, 8)
  for _ in range(100):
    assert update(0, 0, 0)  # recorded false-recovery state
  assert update(1, 2, 0)  # invalid Available must not clear it
  assert update(2, 0, 0)  # initializing Active must not clear it
  assert update(1, 1, 8)  # valid Available with an abort is still faulted
  assert not update(1, 1, 0)
  assert update(3, 1, 6)
  assert not update(2, 1, 0)
