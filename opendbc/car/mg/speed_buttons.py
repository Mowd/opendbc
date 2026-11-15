"""
Fork feature (MG HS/ZS): move the stock ACC set speed with the cruise +/- switches.

openpilot doesn't control gas and brakes on these cars, the radar's ACC does. CC.cruiseControl.speedTarget asks
for a lower set speed (slowing down for a curve); this lowers the stock ACC set speed with -/+ taps and restores
the driver's own set speed afterwards. The radar keeps doing distance keeping and AEB.

Stock switch behavior (MG HS and ZS rlogs): a tap acts ~20-70 ms after it is released and moves the set speed
to the next multiple of 5 km/h (63 -> 60, 33 -> 35); holding a switch steps 1 km/h every 0.2 s after ~0.6 s,
so only taps are used. The switches do nothing in Standby, and the lowest set speed is 30 km/h.
"""
import math

from opendbc.car.carlog import carlog

ACC_ACTIVE = 2
ACC_ENGAGED = (2, 3, 5, 6)  # Active, Override, Standstill Active/Wait

STEP_KPH = 5
MIN_SET_SPEED_KPH = 30

TAP_FRAMES = 4  # gateway cruise switch frames (~30 ms each) held per tap, like RES
TAP_TIMEOUT_FRAMES = 17  # ~0.5 s for the set speed to change before the next tap
MAX_IGNORED_TAPS = 3  # taps in a row that didn't change the set speed before giving up for the drive
DRIVER_HOLDOFF_FRAMES = 200  # 2 s (100 Hz control frames) after the driver touches a cruise switch

SPEED_DOWN = "CCSwStsSpdDecSwA_h2HSC2"
SPEED_UP = "CCSwStsSpdIncSwA_h2HSC2"


def floor_step(kph: float) -> int:
  return int(math.floor(kph / STEP_KPH + 1e-6)) * STEP_KPH


class SpeedButtons:
  def __init__(self):
    self.driver_set_kph = 0  # the set speed the driver chose, restored after a curve
    self.lowered = False  # the set speed is currently below the driver's because of a target
    self.driver_override = False  # the driver changed the speed while it was lowered: leave it until the target clears
    self.last_driver_frame = -DRIVER_HOLDOFF_FRAMES
    self.want = None

    self.tap_button = None
    self.tap_frames = 0
    self.tap_start_kph = 0
    self.ignored_taps = 0
    self.disabled = False  # an unexpected reaction to a tap: stop tapping for the rest of the drive

  def update(self, frame: int, enabled: bool, acc_state: int, set_kph: float, target_kph: float, driver_pressing: bool,
             v_ego_kph: float):
    """Every control frame (100 Hz): decide which switch is wanted, if any."""
    set_kph = round(set_kph)
    self.want = None
    if driver_pressing:
      self.last_driver_frame = frame

    if not enabled or acc_state not in ACC_ENGAGED:
      self.lowered = False
      self.driver_override = False
      self.driver_set_kph = set_kph
      return

    if frame - self.last_driver_frame < DRIVER_HOLDOFF_FRAMES:
      # the driver is setting the speed: that is the new speed to go back to, and don't fight it in this curve
      if self.lowered:
        self.driver_override = True
      self.lowered = False
      self.driver_set_kph = set_kph
      return

    if not self.lowered:
      self.driver_set_kph = set_kph

    curve = 0 < target_kph < self.driver_set_kph
    if not curve:
      self.driver_override = False

    # only tap while the ACC is cruising (not with the gas pressed or stopped)
    if acc_state != ACC_ACTIVE:
      return

    if curve and not self.driver_override:
      desired = max(MIN_SET_SPEED_KPH, floor_step(target_kph))
      # start only when the car is faster than that: following a slower lead or crawling through a turn,
      # lowering the set speed changes nothing yet
      if desired < set_kph and (self.lowered or v_ego_kph > desired):
        self.lowered = True
        self.want = SPEED_DOWN
      elif self.lowered and desired > set_kph:
        self.want = SPEED_UP
    elif self.lowered:
      # back to the driver's speed; a tap only reaches multiples of 5, so stay at or below it
      restore = max(MIN_SET_SPEED_KPH, floor_step(self.driver_set_kph))
      if set_kph < restore:
        self.want = SPEED_UP
      else:
        self.lowered = False

  def reset_tap(self):
    self.tap_frames = 0

  def button(self, set_kph: float):
    """Every new gateway cruise switch frame (~33 Hz): the switch to send with it, or None."""
    set_kph = round(set_kph)
    if self.tap_frames:
      self.tap_frames += 1
      if self.tap_frames <= TAP_FRAMES:
        return self.tap_button
      changed = set_kph != self.tap_start_kph
      if changed or self.tap_frames > TAP_FRAMES + TAP_TIMEOUT_FRAMES:
        self._check_tap(set_kph - self.tap_start_kph)
        self.tap_frames = 0
      return None

    if self.want is not None and not self.disabled:
      self.tap_button = self.want
      self.tap_frames = 1
      self.tap_start_kph = set_kph
      return self.tap_button
    return None

  def _check_tap(self, step: int):
    expected_sign = -1 if self.tap_button == SPEED_DOWN else 1
    if step == 0:
      self.ignored_taps += 1
      if self.ignored_taps >= MAX_IGNORED_TAPS:
        self.disabled = True
        carlog.error(f"MG speed buttons: {self.ignored_taps} taps ignored by the ACC, disabled for this drive")
    elif step * expected_sign < 0 or abs(step) > STEP_KPH:
      self.disabled = True
      carlog.error(f"MG speed buttons: unexpected set speed change {step} km/h after {self.tap_button}, disabled for this drive")
    else:
      self.ignored_taps = 0
