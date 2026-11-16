import math
import unittest

from opendbc.car.mg.speed_buttons import SpeedButtons, SPEED_DOWN, SPEED_UP, DRIVER_HOLDOFF_FRAMES, IGNORED_PAUSE_FRAMES, \
                                         MIN_SET_SPEED_KPH, UP_AFTER_DOWN_FRAMES, UP_MARGIN_KPH

ACTIVE, STANDBY, OVERRIDE = 2, 1, 3
GATEWAY_EVERY = 3  # a gateway cruise switch frame every ~30 ms, control runs at 100 Hz


class FakeAcc:
  """The stock ACC as seen in MG HS/ZS rlogs: a released tap moves the set speed to the next multiple of 5."""
  def __init__(self, set_kph, double_step=False, ignore=False, wrong_way=False):
    self.set_kph = set_kph
    self.pressed = None
    self.double_step = double_step
    self.ignore = ignore
    self.wrong_way = wrong_way

  def frame(self, button):
    if self.pressed is not None and button is None and not self.ignore:
      for _ in range(2 if self.double_step else 1):
        if (self.pressed == SPEED_DOWN) != self.wrong_way:
          self.set_kph = max(MIN_SET_SPEED_KPH, math.ceil(self.set_kph / 5 - 1) * 5)
        else:
          self.set_kph = math.floor(self.set_kph / 5 + 1) * 5
    self.pressed = button


class TestSpeedButtons(unittest.TestCase):
  def run_drive(self, sb, acc, targets, acc_state=ACTIVE, driver_frames=(), v_ego=100):
    """targets: list of (control frames, target km/h). Returns the set speed after each part."""
    frame = 0
    out = []
    for n, target in targets:
      for _ in range(n):
        driver = frame in driver_frames
        sb.update(frame, True, acc_state, acc.set_kph, target, driver, v_ego)
        if frame % GATEWAY_EVERY == 0:
          button = None if driver else sb.button(acc.set_kph)
          acc.frame(button)
        frame += 1
      out.append(acc.set_kph)
    return out

  def test_no_target_no_taps(self):
    sb, acc = SpeedButtons(), FakeAcc(60)
    self.assertEqual(self.run_drive(sb, acc, [(500, 0), (500, 200)]), [60, 60])

  def test_lower_and_restore(self):
    sb, acc = SpeedButtons(), FakeAcc(60)
    # curve asks for 42 km/h: 60 -> 40, then back to 60 once it clears
    self.assertEqual(self.run_drive(sb, acc, [(300, 42), (300, 0)]), [40, 60])
    self.assertFalse(sb.lowered)
    self.assertFalse(sb.disabled)

  def test_restore_never_above_driver(self):
    sb, acc = SpeedButtons(), FakeAcc(63)
    self.assertEqual(self.run_drive(sb, acc, [(300, 50), (300, 0)]), [50, 60])

  def test_min_set_speed(self):
    sb, acc = SpeedButtons(), FakeAcc(50)
    self.assertEqual(self.run_drive(sb, acc, [(400, 18)]), [30])
    self.assertFalse(sb.disabled)

  def test_rising_target_steps_up(self):
    sb, acc = SpeedButtons(), FakeAcc(70)
    self.assertEqual(self.run_drive(sb, acc, [(300, 40), (300, 55 + UP_MARGIN_KPH), (300, 90)]), [40, 55, 70])

  def test_step_hysteresis(self):
    # just below a step keeps it (44 -> 45, not 40); going back up needs the target well above the next step
    sb, acc = SpeedButtons(), FakeAcc(70)
    self.assertEqual(self.run_drive(sb, acc, [(300, 44), (400, 52), (400, 43.5), (400, 53.5)]), [45, 45, 45, 50])

  def test_curves_one_after_another(self):
    # MG HS route 23: a target wandering 44-53 km/h through curves at ~40 km/h moved the set speed ten times
    sb, acc = SpeedButtons(), FakeAcc(70)
    taps = 0
    frame = 0
    for i in range(1700):
      target = 48.5 + 4.5 * math.sin(i / 120)
      sb.update(frame, True, ACTIVE, acc.set_kph, target, False, 40)
      if frame % GATEWAY_EVERY == 0:
        b = sb.button(acc.set_kph)
        taps += b is not None and sb.tap_frames == 1
        acc.frame(b)
      frame += 1
    self.assertLessEqual(taps, 6)  # 70 -> 45 once, then at most one step back and forth

  def test_up_without_waiting_for_the_car(self):
    # HS route 2a: waiting for the car to reach each 5 km/h step made it crawl back up from 30 km/h after a turn
    sb, acc = SpeedButtons(), FakeAcc(70)
    frame = 0

    def drive(frames, target, v_ego):
      nonlocal frame
      for _ in range(frames):
        sb.update(frame, True, ACTIVE, acc.set_kph, target, False, v_ego(acc.set_kph))
        if frame % GATEWAY_EVERY == 0:
          acc.frame(sb.button(acc.set_kph))
        frame += 1
      return acc.set_kph

    self.assertEqual(drive(300, 40, lambda s: 50), 40)
    # the target is back up to 60 while the car is still at 30: up to 55 right away (60 - UP_MARGIN_KPH -> 55)
    self.assertEqual(drive(800, 60, lambda s: 30), 55)

  def test_not_when_already_slower(self):
    # following a slow lead at 35 km/h with 60 set: a 40 km/h curve target changes nothing yet
    sb, acc = SpeedButtons(), FakeAcc(60)
    self.assertEqual(self.run_drive(sb, acc, [(300, 40)], v_ego=35), [60])
    self.assertFalse(sb.lowered)

  def test_not_while_override_or_standby(self):
    for state in (OVERRIDE, STANDBY):
      sb, acc = SpeedButtons(), FakeAcc(60)
      self.assertEqual(self.run_drive(sb, acc, [(300, 40)], acc_state=state), [60])

  def test_driver_takes_over(self):
    sb, acc = SpeedButtons(), FakeAcc(60)
    self.run_drive(sb, acc, [(300, 40)])
    self.assertEqual(acc.set_kph, 40)
    # the driver taps + during the curve: their speed now, no more taps in this curve
    acc.set_kph = 45
    frame0 = 300
    sb.update(frame0, True, ACTIVE, 45, 40, True, 50)
    for f in range(frame0 + 1, frame0 + DRIVER_HOLDOFF_FRAMES + 300):
      sb.update(f, True, ACTIVE, acc.set_kph, 40, False, 50)
      if f % GATEWAY_EVERY == 0:
        acc.frame(sb.button(acc.set_kph))
    self.assertEqual(acc.set_kph, 45)
    self.assertFalse(sb.lowered)

  def test_disabled_on_wrong_way(self):
    sb, acc = SpeedButtons(), FakeAcc(60, wrong_way=True)
    self.run_drive(sb, acc, [(300, 50)])
    self.assertTrue(sb.disabled)
    self.assertEqual(acc.set_kph, 65)  # one tap the wrong way, then nothing more

  def test_paused_on_double_step(self):
    # the right way but two steps: pause, don't give up for the drive
    sb, acc = SpeedButtons(), FakeAcc(60, double_step=True)
    self.run_drive(sb, acc, [(300, 50)])
    self.assertFalse(sb.disabled)
    self.assertEqual(acc.set_kph, 50)
    self.assertGreater(sb.paused_until, 0)
    acc.double_step = False
    self.assertEqual(self.run_drive(sb, acc, [(IGNORED_PAUSE_FRAMES + 400, 37)]), [35])

  def test_disabled_past_driver_speed(self):
    # a double step up past the driver's own speed: the car would go faster than the driver chose, stop for the drive
    sb, acc = SpeedButtons(), FakeAcc(60)
    self.assertEqual(self.run_drive(sb, acc, [(300, 57)]), [55])
    acc.double_step = True
    self.assertEqual(self.run_drive(sb, acc, [(UP_AFTER_DOWN_FRAMES + 300, 0)]), [65])
    self.assertTrue(sb.disabled)

  def test_gas_during_tap(self):
    # MG ZS route 16: the driver pressed the gas during a tap up that restored the speed after a lead (Override), and
    # the set speed went 35 -> 50 at once. That isn't the ACC misbehaving: keep going once it is Active again.
    sb, acc = SpeedButtons(), FakeAcc(110)
    self.assertEqual(self.run_drive(sb, acc, [(800, 27)], v_ego=60), [30])
    frame, state, gassed = 800, ACTIVE, False
    for _ in range(1500):
      if sb.tap_frames == 3 and not gassed and acc.set_kph == 35:
        state, gassed = OVERRIDE, True
        acc.set_kph = 50  # Override: the tap moves it by three steps
      elif gassed and frame > 1000:
        state = ACTIVE
      sb.update(frame, True, state, acc.set_kph, 0, False, 30)
      if frame % GATEWAY_EVERY == 0:
        acc.frame(sb.button(acc.set_kph))
      frame += 1
    self.assertTrue(gassed)
    self.assertFalse(sb.disabled)
    self.assertEqual(sb.paused_until, 0)
    self.assertEqual(acc.set_kph, 110)

  def test_paused_when_ignored(self):
    # the ACC ignores taps for a while: pause, don't give up for the drive
    sb, acc = SpeedButtons(), FakeAcc(60, ignore=True)
    self.run_drive(sb, acc, [(400, 40)])
    self.assertFalse(sb.disabled)
    self.assertGreater(sb.paused_until, 0)
    # it takes taps again: after the pause, the curve is followed
    acc.ignore = False
    self.assertEqual(self.run_drive(sb, acc, [(IGNORED_PAUSE_FRAMES + 400, 40)]), [40])

  def test_no_quick_reversal(self):
    # a target flickering across a 5 km/h step: one tap down, no tap back up right after
    sb, acc = SpeedButtons(), FakeAcc(60)
    ups = []
    frame = 0
    for i in range(UP_AFTER_DOWN_FRAMES - 50):
      target = 51 if i < 30 or (i // 20) % 2 else 58  # 51 -> 50, 58 -> 55
      sb.update(frame, True, ACTIVE, acc.set_kph, target, False, 100)
      if frame % GATEWAY_EVERY == 0:
        b = sb.button(acc.set_kph)
        ups.append(b == SPEED_UP)
        acc.frame(b)
      frame += 1
    self.assertEqual(acc.set_kph, 50)
    self.assertFalse(any(ups))

  def test_disengaged_resets(self):
    sb, acc = SpeedButtons(), FakeAcc(60)
    self.run_drive(sb, acc, [(300, 40)])
    sb.update(1000, False, STANDBY, acc.set_kph, 40, False, 50)
    self.assertFalse(sb.lowered)
    self.assertIsNone(sb.want)


if __name__ == "__main__":
  unittest.main()
