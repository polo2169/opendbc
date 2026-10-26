"""Exercise actual P87 CAN output against the compiled Panda safety hook."""
import math
import unittest

from opendbc.car import structs
from opendbc.car.psa.tests import test_5008 as host_fixture
from opendbc.car.psa.values import p87_torque_limit
from opendbc.safety.tests.common import make_msg
from opendbc.safety.tests import test_psa_5008 as native_fixture


class TestP87TorqueCurve(unittest.TestCase):
  def test_knots_clamping_rounding_and_nonfinite_input(self):
    for speed, ceiling in ((30., 70), (49.99, 70), (50., 70), (50.01, 69),
                           (70., 60), (90., 50), (110., 40), (130., 30), (140., 30)):
      with self.subTest(speed=speed):
        self.assertEqual(p87_torque_limit(speed), ceiling)
    for speed in (math.nan, math.inf, -math.inf):
      self.assertEqual(p87_torque_limit(speed), 0)

  def test_native_accepts_exact_ceiling_and_rejects_one_more_at_every_centi_kph(self):
    native = native_fixture.TestP87Safety()
    native.setUp()
    native.feed(speed=50.)
    for speed_centi in range(5000, 14001):
      ceiling = max(30, min(70, (19000 - speed_centi) // 200))
      self.assertEqual(p87_torque_limit(speed_centi / 100), ceiling)
      wheels = speed_centi.to_bytes(2, 'big') * 4
      self.assertTrue(native.safety.safety_rx_hook(make_msg(0, 0x30D, dat=wheels)))
      for sign in (-1, 1):
        native.safety.set_desired_torque_last(sign * ceiling)
        native.safety.set_rt_torque_last(sign * ceiling)
        self.assertTrue(native.safety.safety_tx_hook(native.command(sign * ceiling)), (speed_centi, sign))
        self.assertFalse(native.safety.safety_tx_hook(native.command(sign * (ceiling + 1))), (speed_centi, sign))

  def run_controller(self, speeds, sign=1):
    helper = host_fixture.TestP87Interface()
    ci = helper.interface(True)
    native = native_fixture.TestP87Safety()
    native.setUp()
    cc = structs.CarControl(latActive=True, enabled=True)
    cc.actuators.torque = float(sign)
    outputs = []
    for step, speed in enumerate(speeds):
      now = 1_000_000_000 + step * 50_000_000
      ci.CC.frame = step * ci.CC.params.STEER_STEP
      native.safety.set_timer(100_000 + step * 50_000)
      native.feed(speed=speed)
      state = helper.state(speed_kph=speed, p87_lka_timestamp=now, p87_eps_timestamp=now)
      act, sends = ci.CC.update(cc.as_reader(), structs.CarControlSP(), state, now)
      self.assertEqual(len(sends), 1)
      self.assertLessEqual(abs(act.torqueOutputCan), p87_torque_limit(speed))
      self.assertEqual(ci.CC.p87_lateral_status['torque_limit_raw'], p87_torque_limit(speed))
      self.assertAlmostEqual(act.torque, act.torqueOutputCan / 150.)
      encoded = (sends[0].dat[3] << 3) | (sends[0].dat[4] >> 5)
      encoded = encoded - 2048 if encoded >= 1024 else encoded
      self.assertEqual(encoded, act.torqueOutputCan)
      self.assertTrue(native.safety.safety_tx_hook(make_msg(sends[0].src, sends[0].address, dat=sends[0].dat)),
                      (step, speed, act.torqueOutputCan))
      outputs.append(act.torqueOutputCan)
    return outputs

  def test_both_directions_reach_each_ceiling_through_real_controller_and_native_safety(self):
    for speed, ceiling in ((30., 70), (50., 70), (70., 60), (90., 50), (110., 40), (130., 30), (140., 30)):
      for sign in (-1, 1):
        with self.subTest(speed=speed, sign=sign):
          self.assertEqual(self.run_controller([speed] * 12, sign)[-1], sign * ceiling)

  def test_acceleration_and_deceleration_track_ceiling_without_native_rejections(self):
    speeds = [50.] * 12 + [v / 2 for v in range(101, 261)] + [v / 2 for v in range(259, 99, -1)]
    for sign in (-1, 1):
      outputs = self.run_controller(speeds, sign)
      for speed, torque in zip(speeds[12:], outputs[12:], strict=True):
        self.assertEqual(torque, sign * p87_torque_limit(speed))

  def test_fractional_wheel_average_rounds_controller_conservatively(self):
    for total in (20001, 20003, 20799, 20801, 51999, 52001):
      speed = total / 400
      native_ceiling = max(30, min(70, (19000 - total // 4) // 200))
      self.assertLessEqual(p87_torque_limit(speed), native_ceiling)
      self.assertLessEqual(native_ceiling - p87_torque_limit(speed), 1)


if __name__ == '__main__':
  unittest.main()
