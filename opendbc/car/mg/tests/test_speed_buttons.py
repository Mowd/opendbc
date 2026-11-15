import math
import unittest

from opendbc.car.mg.speed_buttons import SpeedButtons, SPEED_DOWN, DRIVER_HOLDOFF_FRAMES, MIN_SET_SPEED_KPH

ACTIVE, STANDBY, OVERRIDE = 2, 1, 3
GATEWAY_EVERY = 3  # a gateway cruise switch frame every ~30 ms, control runs at 100 Hz


class FakeAcc:
  """The stock ACC as seen in MG HS/ZS rlogs: a released tap moves the set speed to the next multiple of 5."""
  def __init__(self, set_kph, double_step=False, ignore=False):
    self.set_kph = set_kph
    self.pressed = None
    self.double_step = double_step
    self.ignore = ignore

  def frame(self, button):
    if self.pressed is not None and button is None and not self.ignore:
      for _ in range(2 if self.double_step else 1):
        if self.pressed == SPEED_DOWN:
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
    self.assertEqual(self.run_drive(sb, acc, [(300, 40), (300, 55), (300, 90)]), [40, 55, 70])

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

  def test_disabled_on_unexpected_step(self):
    sb, acc = SpeedButtons(), FakeAcc(60, double_step=True)
    self.run_drive(sb, acc, [(300, 50)])
    self.assertTrue(sb.disabled)
    self.assertEqual(acc.set_kph, 50)  # one double step, then nothing more

  def test_disabled_when_ignored(self):
    sb, acc = SpeedButtons(), FakeAcc(60, ignore=True)
    self.run_drive(sb, acc, [(1000, 40)])
    self.assertTrue(sb.disabled)

  def test_disengaged_resets(self):
    sb, acc = SpeedButtons(), FakeAcc(60)
    self.run_drive(sb, acc, [(300, 40)])
    sb.update(1000, False, STANDBY, acc.set_kph, 40, False, 50)
    self.assertFalse(sb.lowered)
    self.assertIsNone(sb.want)


if __name__ == "__main__":
  unittest.main()
