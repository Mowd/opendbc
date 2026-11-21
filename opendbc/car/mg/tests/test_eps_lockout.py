from types import SimpleNamespace

from opendbc.car.mg.carcontroller import (EPS_FOLLOWING, EPS_FREEZE_FRAMES, EPS_LOCKOUT, EPS_RELEASE_MAX_FRAMES,
                                          EPS_RELEASE_MIN_FRAMES, EPS_RELEASED, EPS_RETRY_FRAMES, EPS_STUCK_FRAMES,
                                          CarController)


def controller(apply_torque_last=0):
  cc = CarController.__new__(CarController)
  cc.eps_releasing = False
  cc.eps_release_frames = 0
  cc.eps_retry_frames = 0
  cc.eps_resp_last = 0.0
  cc.eps_frozen_frames = 0
  cc.eps_stuck_frames = 0
  cc.apply_torque_last = apply_torque_last
  return cc


def eps(sts=EPS_FOLLOWING, resp=0.0, driver=0.0):
  return SimpleNamespace(eps_ctrl_sts=sts, out=SimpleNamespace(steeringTorqueEps=resp, steeringTorque=driver))


def run(cc, CS, frames, lat_active=True):
  out = []
  for _ in range(frames):
    ok = cc.eps_request_ok(CS, lat_active)
    if not ok:
      cc.apply_torque_last = 0  # carcontroller sends no torque while the request is dropped
    out.append(ok)
  return out


class TestEpsLockout:
  def test_requests_while_following(self):
    cc = controller(100)
    assert all(cc.eps_request_ok(eps(resp=0.5 + 0.01 * i), True) for i in range(300))

  def test_lockout_waits_for_the_release(self):
    # MG ZS route 3d: locked out while requested, released once the request was dropped
    cc = controller(-200)
    assert not any(run(cc, eps(EPS_LOCKOUT), 30))
    # still following-without-output: keep waiting, a fixed short pause wasn't enough on route 41
    assert not any(run(cc, eps(EPS_FOLLOWING, -0.5), 10))
    assert run(cc, eps(EPS_RELEASED), 1) == [True]
    assert all(run(cc, eps(EPS_FOLLOWING, 0.0), 10))

  def test_release_takes_at_least_the_minimum(self):
    cc = controller(-200)
    out = run(cc, eps(EPS_LOCKOUT), 1) + run(cc, eps(EPS_RELEASED), EPS_RELEASE_MIN_FRAMES + 1)
    assert not any(out[:EPS_RELEASE_MIN_FRAMES]) and out[EPS_RELEASE_MIN_FRAMES]

  def test_retries_while_locked(self):
    # some MG HS lockouts only released once a request was sent again: alternate dropping and requesting
    cc = controller(-200)
    cycle = [False] * EPS_RELEASE_MAX_FRAMES + [True] * (EPS_RETRY_FRAMES + 1)
    assert run(cc, eps(EPS_LOCKOUT), 2 * len(cycle)) == cycle * 2

  def test_not_without_lat_active(self):
    cc = controller()
    assert all(run(cc, eps(EPS_LOCKOUT), 50, lat_active=False))


class TestEpsFreeze:
  def test_route_3d_freeze(self):
    # MG ZS route 3d seg 12: output stuck at -0.87 Nm, request -1.33 Nm
    cc = controller(-133)
    out = [cc.eps_request_ok(eps(resp=-0.87, driver=0.9), True) for _ in range(EPS_FREEZE_FRAMES + 2)]
    assert all(out[:EPS_FREEZE_FRAMES]) and not out[-1]

  def test_route_41_freeze_after_the_driver_let_go(self):
    # MG ZS route 41 seg 2: frozen at -2.52 Nm in a curve, request -3 Nm, driver no longer steering
    cc = controller(-300)
    out = [cc.eps_request_ok(eps(resp=-2.52, driver=-0.1), True) for _ in range(EPS_FREEZE_FRAMES + 2)]
    assert not out[-1]

  def test_not_steady_or_small(self):
    cc = controller(-90)  # output matches the request: a steady request
    assert all(cc.eps_request_ok(eps(resp=-0.87), True) for _ in range(100))
    cc = controller(-60)  # small output
    assert all(cc.eps_request_ok(eps(resp=-0.2), True) for _ in range(100))


class TestEpsStuck:
  def test_route_41_following_without_output(self):
    # MG ZS route 41: the EPS reported following but gave no torque for 7-29 s while requested up to 3 Nm
    cc = controller(-150)
    out = [cc.eps_request_ok(eps(resp=0.0), True) for _ in range(EPS_STUCK_FRAMES + 1)]
    assert all(out[:EPS_STUCK_FRAMES - 1]) and not out[-1]

  def test_not_with_a_small_request(self):
    cc = controller(-30)
    assert all(cc.eps_request_ok(eps(resp=0.0), True) for _ in range(200))
