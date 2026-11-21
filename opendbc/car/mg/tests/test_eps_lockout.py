from opendbc.car.mg.carcontroller import EPS_PAUSE_FRAMES, EPS_RETRY_FRAMES, CarController


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
