import unittest
from types import SimpleNamespace

from opendbc.car import Bus, structs
from opendbc.car.can_definitions import CanData
from opendbc.car.fingerprints import (
  all_legacy_fingerprint_cars,
  eliminate_incompatible_cars,
)
from opendbc.car.psa.fingerprints import FINGERPRINTS
from opendbc.car.psa.interface import CarInterface
from opendbc.car.psa.psacan import psa_checksum
from opendbc.car.psa.values import CAR, DBC


# One coherent stationary snapshot from the read-only capture
# learn-20260811T170039Z-e1fe3ff3. These are RX frames, not command fixtures.
STATIONARY_CAPTURE = {
  0x208: "1AC133004C333233",
  0x2F5: "BB00007FFF0001",
  0x305: "14E30007E10020",
  0x30D: "0000000000000000",
  0x348: "00219F4311036059",
  0x3AD: "500000018072008E",
  0x3CD: "FFFD00000B226040",
  0x412: "10000000007C0000",
  0x452: "000000030000",
  0x50E: "022600045E42FF2D",
  0x572: "98807FC088000001",
}


class TestPeugeot308T9(unittest.TestCase):
  def setUp(self):
    self.CP = CarInterface.get_non_essential_params(CAR.PSA_PEUGEOT_308_T9)
    self.CI = CarInterface(self.CP)

  def update_from_stationary_capture(self, overrides=None):
    capture = STATIONARY_CAPTURE | (overrides or {})
    frames = [
      CanData(address, bytes.fromhex(data_hex), 0)
      for address, data_hex in capture.items()
    ]
    return self.CI.update([(1_000_000_000, frames)])

  def test_platform_is_hard_read_only(self):
    self.assertTrue(self.CP.dashcamOnly)
    self.assertEqual(
      self.CP.safetyConfigs[0].safetyModel,
      structs.CarParams.SafetyModel.noOutput,
    )
    self.assertFalse(self.CP.openpilotLongitudinalControl)
    self.assertFalse(self.CP.alphaLongitudinalAvailable)
    self.assertTrue(self.CP.pcmCruise)
    self.assertFalse(self.CP.autoResumeSng)

  def test_uses_vehicle_specific_dbc_and_fingerprint(self):
    self.assertEqual(
      DBC[CAR.PSA_PEUGEOT_308_T9][Bus.pt],
      "peugeot_308_t9_2018",
    )
    fingerprint = FINGERPRINTS[CAR.PSA_PEUGEOT_308_T9][0]
    for address, data_hex in STATIONARY_CAPTURE.items():
      self.assertEqual(fingerprint[address], len(bytes.fromhex(data_hex)))

  def test_recorded_snapshot_fingerprints_as_t9(self):
    candidates = all_legacy_fingerprint_cars()
    for address, data_hex in STATIONARY_CAPTURE.items():
      candidates = eliminate_incompatible_cars(
        CanData(address, bytes.fromhex(data_hex), 0),
        candidates,
      )

    self.assertEqual(candidates, [CAR.PSA_PEUGEOT_308_T9])

  def test_decodes_recorded_stationary_carstate(self):
    state = self.update_from_stationary_capture()

    self.assertAlmostEqual(state.vEgoRaw, 0.0)
    self.assertTrue(state.standstill)
    self.assertAlmostEqual(state.steeringAngleDeg, 534.7, places=3)
    self.assertAlmostEqual(state.steeringRateDeg, 0.0)
    self.assertAlmostEqual(state.steeringTorque, 0.0)
    self.assertFalse(state.steeringPressed)
    self.assertFalse(state.gasPressed)
    self.assertFalse(state.brakePressed)
    self.assertTrue(state.parkingBrake)
    self.assertFalse(state.doorOpen)
    self.assertFalse(state.seatbeltUnlatched)
    self.assertEqual(state.gearShifter, structs.CarState.GearShifter.neutral)
    self.assertTrue(state.cruiseState.available)
    self.assertFalse(state.cruiseState.enabled)
    self.assertTrue(state.cruiseState.nonAdaptive)
    self.assertAlmostEqual(state.cruiseState.speed, 0.0)

  def test_eat6_mapping_matches_obd_replay(self):
    drive = self.update_from_stationary_capture({
      0x348: "04274E3D11036059",
    })
    self.assertEqual(drive.gearShifter, structs.CarState.GearShifter.drive)

    reverse = self.update_from_stationary_capture({
      0x348: "09274E3D11036059",
    })
    self.assertEqual(reverse.gearShifter, structs.CarState.GearShifter.reverse)

  def test_parking_brake_uses_validated_esp_state(self):
    released = self.update_from_stationary_capture({
      0x3AD: "000000007EE10054",
    })
    self.assertFalse(released.parkingBrake)

    self.CI = CarInterface(self.CP)
    applied = self.update_from_stationary_capture({
      0x3AD: "50000001805200EA",
    })
    self.assertTrue(applied.parkingBrake)

  def test_driver_body_and_rvv_signals(self):
    state = self.update_from_stationary_capture({
      # Accelerator > 0 and persistent RVV state 2.
      0x208: "1AC1332048333233",
      # Brake pedal plus driver and rear-left doors open.
      0x412: "30000000007C2800",
      # Driver seatbelt state 1 (unlatched).
      0x572: "58807FC088000001",
    })

    self.assertTrue(state.gasPressed)
    self.assertTrue(state.brakePressed)
    self.assertTrue(state.doorOpen)
    self.assertTrue(state.seatbeltUnlatched)
    self.assertTrue(state.cruiseState.available)
    self.assertTrue(state.cruiseState.enabled)
    self.assertTrue(state.cruiseState.nonAdaptive)

  def test_vehicle_validated_psa_checksums(self):
    # One real payload per protected message. The complete 72-session corpus
    # and pass counts are recorded beside the DBC in the OBD repository.
    samples = {
      # address: (payload, checksum start bit, transmitted checksum)
      0x228: ("323000CEFEC33200", 24, 0xE),
      0x2ED: ("11000000000000", 7, 0x1),
      0x2F5: ("9D00007FFF0001", 7, 0x9),
      0x329: ("9C102140", 7, 0x9),
      0x349: ("0CFE0000FEA081FE", 55, 0x8),
      0x38D: ("00000000AF3E7600", 43, 0xE),
      0x3AD: ("50000001805200EA", 59, 0xA),
      0x3C9: ("3235F07821A60000", 31, 0x7),
      0x3CD: ("000100000B33B17E", 63, 0x7),
      0x42D: ("FE00C300", 20, 0xC),
      0x50D: ("00000000008B8C05", 59, 0x5),
    }

    for address, (payload, start_bit, expected) in samples.items():
      with self.subTest(address=hex(address)):
        signal = SimpleNamespace(start_bit=start_bit)
        self.assertEqual(
          psa_checksum(address, signal, bytearray.fromhex(payload)),
          expected,
        )

  def test_controller_never_emits_can(self):
    self.update_from_stationary_capture()
    control = structs.CarControl()
    control.enabled = True
    control.latActive = True
    control.longActive = True
    control.actuators.steeringAngleDeg = 90.0
    control.actuators.torque = 1.0
    control = control.as_reader()

    for frame in range(100):
      _, can_sends = self.CI.apply(control, frame * 10_000_000)
      self.assertEqual(can_sends, [])


if __name__ == "__main__":
  unittest.main()
