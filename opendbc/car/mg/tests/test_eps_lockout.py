from opendbc.car.mg.carcontroller import EPS_FREEZE_FRAMES, EPS_PAUSE_FRAMES, EPS_RETRY_FRAMES, CarController


def controller():
  cc = CarController.__new__(CarController)
  cc.eps_pause_frames = 0
  cc.eps_retry_frames = 0
  return cc


def run(cc, locked, frames):
  return [cc.eps_lockout_ok(locked) for _ in range(frames)]


class TestEpsLockout:
  def test_requests_while_following(self):
    assert all(run(controller(), False, 200))

  def test_pause_then_request_again(self):
    # MG ZS route 3d: the EPS stays locked while requested, clears once the request is dropped
    cc = controller()
    assert not any(run(cc, True, EPS_PAUSE_FRAMES))
    assert all(run(cc, False, 200))

  def test_keeps_retrying_while_locked(self):
    # some MG HS lockouts only cleared once a request was sent again: alternate pausing and requesting
    cc = controller()
    out = run(cc, True, 3 * (EPS_PAUSE_FRAMES + EPS_RETRY_FRAMES))
    assert out == ([False] * EPS_PAUSE_FRAMES + [True] * EPS_RETRY_FRAMES) * 3

  def test_lockout_cleared_while_requesting_again(self):
    cc = controller()
    run(cc, True, EPS_PAUSE_FRAMES + 5)
    assert all(run(cc, False, 200))
    # a new lockout later pauses again right away
    assert not cc.eps_lockout_ok(True)


class FakeCS:
  def __init__(self, resp, driver, ctl=1):
    self.eps_ctrl_sts = ctl
    self.out = type('Out', (), {'steeringTorqueEps': resp, 'steeringTorque': driver})()


def frozen_controller(apply_torque_last):
  cc = controller()
  cc.eps_resp_last = 0.0
  cc.eps_frozen_frames = 0
  cc.apply_torque_last = apply_torque_last
  return cc


class TestEpsFreeze:
  def test_route_3d_freeze(self):
    # MG ZS route 3d seg 12: output stuck at -0.87 Nm, request -1.33 Nm, driver +0.9 Nm the other way
    cc = frozen_controller(-133)
    out = [cc.eps_frozen(FakeCS(-0.87, 0.9)) for _ in range(EPS_FREEZE_FRAMES + 2)]
    assert not any(out[:EPS_FREEZE_FRAMES]) and out[-1]

  def test_not_while_following(self):
    # the output tracks a changing request
    cc = frozen_controller(-133)
    assert not any(cc.eps_frozen(FakeCS(-0.5 - 0.01 * i, 0.9)) for i in range(100))

  def test_not_steady_or_without_the_driver(self):
    cc = frozen_controller(-90)  # output matches the request: just a steady request
    assert not any(cc.eps_frozen(FakeCS(-0.87, 0.9)) for _ in range(100))
    cc = frozen_controller(-133)  # driver not steering against it
    assert not any(cc.eps_frozen(FakeCS(-0.87, -0.9)) for _ in range(100))
    assert not any(cc.eps_frozen(FakeCS(-0.87, 0.3)) for _ in range(100))
    cc = frozen_controller(-133)  # locked out already: handled as EPS_LOCKOUT
    assert not any(cc.eps_frozen(FakeCS(0.0, 0.9, ctl=4)) for _ in range(100))
