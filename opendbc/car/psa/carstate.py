from opendbc.car import structs, Bus
from opendbc.can.parser import CANParser
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.psa.values import DBC, CAR, CarControllerParams
from opendbc.car.interfaces import CarStateBase

GearShifter = structs.CarState.GearShifter

class CarState(CarStateBase):
  def update(self, can_parsers) -> structs.CarState:
    if self.CP.carFingerprint == CAR.PSA_PEUGEOT_308_T9:
      return self._update_308(can_parsers)
    return self._update_upstream_psa(can_parsers)

  def _update_308(self, can_parsers) -> structs.CarState:
    """
    Peugeot 308 T9 read-only CarState.

    The USB-CAN/replay publishes all observed frames on source bus 0, so one
    parser is used for the initial native integration.
    """
    cp = can_parsers[Bus.main]
    ret = structs.CarState()

    # Four wheel speeds, validated against the user's OBD frontend/replay.
    wheel_fl_kph = cp.vl['T9_WHEEL_SPEEDS_30D']['WheelSpeedFrontLeftKph']
    wheel_fr_kph = cp.vl['T9_WHEEL_SPEEDS_30D']['WheelSpeedFrontRightKph']
    wheel_rl_kph = cp.vl['T9_WHEEL_SPEEDS_30D']['WheelSpeedRearLeftKph']
    wheel_rr_kph = cp.vl['T9_WHEEL_SPEEDS_30D']['WheelSpeedRearRightKph']

    ret.wheelSpeeds.fl = wheel_fl_kph * CV.KPH_TO_MS
    ret.wheelSpeeds.fr = wheel_fr_kph * CV.KPH_TO_MS
    ret.wheelSpeeds.rl = wheel_rl_kph * CV.KPH_TO_MS
    ret.wheelSpeeds.rr = wheel_rr_kph * CV.KPH_TO_MS

    wheel_values = (
      ret.wheelSpeeds.fl,
      ret.wheelSpeeds.fr,
      ret.wheelSpeeds.rl,
      ret.wheelSpeeds.rr,
    )
    ret.vEgoRaw = sum(wheel_values) / 4.0
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = ret.vEgoRaw < 0.1

    ret.yawRate = (
      cp.vl['T9_BRAKE_DYNAMICS_3CD']['YawRateDegS']
      * CV.DEG_TO_RAD
    )

    # Driver inputs.
    ret.gasPressed = (
      cp.vl['T9_ENGINE_DYNAMICS_208']['AcceleratorPositionPct'] > 0
    )
    ret.brakePressed = bool(
      cp.vl['T9_BODY_STATUS_412']['BrakePedalActive']
    )
    # 0x3AD was observed in all four raw states on this vehicle, including
    # 74389 applied samples. The 0x412 bit remains a useful redundant layout
    # reference, but it never became active in the local capture corpus.
    ret.parkingBrake = (
      cp.vl['T9_EASY_MOVE_3AD']['ParkingBrakeState'] == 1
    )

    # Steering: angle/rate are valid on 0x305; driver torque is valid on 0x2F5.
    ret.steeringAngleDeg = (
      cp.vl['T9_STEERING_DYNAMICS_305']['SteeringAngleDeg']
    )
    rate = cp.vl['T9_STEERING_DYNAMICS_305']['SteeringRateMagnitudeDegS']
    rate_sign = cp.vl['T9_STEERING_DYNAMICS_305']['SteeringRateSign']
    ret.steeringRateDeg = rate * (1 if rate_sign == 0 else -1)

    ret.steeringTorque = cp.vl['T9_STEERING_TORQUE_2F5']['DriverTorqueRaw']
    ret.steeringPressed = self.update_steering_pressed(
      abs(ret.steeringTorque) > CarControllerParams.T9_STEER_DRIVER_THRESHOLD_RAW,
      5,
    )

    # The persistent RVV state is the validated two-bit candidate in 0x208;
    # 0x452 only contains activation requests and must not be used as an
    # on/off latch. The selector and setpoint in 0x50E were validated over
    # repeated off/RVV/LVV transitions and five RVV engagements. This vehicle
    # has no ACC configured, so no unobserved R3 ACC status is consumed here.
    xvv_state = cp.vl['T9_ENGINE_DYNAMICS_208']['CruiseStateCandidate']
    cruise_mode = cp.vl['T9_CRUISE_SETPOINT_50E']['CruiseMode']
    ret.cruiseState.enabled = xvv_state == 2 and cruise_mode == 1

    setpoint_raw = cp.vl['T9_CRUISE_SETPOINT_50E']['CruiseSetpointKph']
    ret.cruiseState.speed = (
      setpoint_raw * CV.KPH_TO_MS if setpoint_raw < 255 else 0.0
    )
    ret.cruiseState.available = cruise_mode == 1
    ret.cruiseState.nonAdaptive = (
      cp.vl['T9_DRIVER_CRUISE_COMMAND_452']['LongitudinalRegulationType'] != 3
    )
    ret.cruiseState.standstill = False
    ret.accFaulted = False

    reverse = bool(cp.vl['T9_BODY_STATUS_412']['ReverseGearActive'])
    current_gear = int(cp.vl['T9_ENGINE_GEAR_348']['CurrentGear'])

    # Match the OBD replay's EAT6 coding exactly: 0=N/no forward ratio,
    # 1..6=Drive and 9=Reverse. The independent 0x412 bit is redundant for R.
    if reverse or current_gear == 9:
      ret.gearShifter = GearShifter.reverse
    elif 1 <= current_gear <= 6:
      ret.gearShifter = GearShifter.drive
    elif current_gear == 0:
      ret.gearShifter = GearShifter.neutral
    else:
      ret.gearShifter = GearShifter.unknown

    # 0x452 TurnSignalStatus: 0=off, 1=right, 2=left, 3=hazards.
    blinker = cp.vl['T9_DRIVER_CRUISE_COMMAND_452']['TurnSignalStatus']
    ret.leftBlinker = blinker in (2, 3)
    ret.rightBlinker = blinker in (1, 3)

    ret.doorOpen = any((
      cp.vl['T9_BODY_STATUS_412']['DriverDoorOpen'],
      cp.vl['T9_BODY_STATUS_412']['PassengerDoorOpen'],
      cp.vl['T9_BODY_STATUS_412']['RearLeftDoorOpen'],
      cp.vl['T9_BODY_STATUS_412']['RearRightDoorOpen'],
    ))
    ret.seatbeltUnlatched = (
      cp.vl['T9_RESTRAINTS_572']['DriverSeatbeltState'] != 2
    )

    return ret

  def _update_upstream_psa(self, can_parsers) -> structs.CarState:
    cp = can_parsers[Bus.main]
    cp_adas = can_parsers[Bus.adas]
    cp_cam = can_parsers[Bus.cam]
    ret = structs.CarState()

    # car speed
    self.parse_wheel_speeds(ret,
      cp.vl['Dyn4_FRE']['P263_VehV_VPsvValWhlFrtL'],
      cp.vl['Dyn4_FRE']['P264_VehV_VPsvValWhlFrtR'],
      cp.vl['Dyn4_FRE']['P265_VehV_VPsvValWhlBckL'],
      cp.vl['Dyn4_FRE']['P266_VehV_VPsvValWhlBckR'],
    )
    ret.yawRate = cp_adas.vl['HS2_DYN_UCF_MDD_32D']['VITESSE_LACET_BRUTE'] * CV.DEG_TO_RAD
    ret.standstill = bool(cp_adas.vl['HS2_DYN_UCF_MDD_32D']['VEHICLE_STANDSTILL'])

    # gas
    ret.gasPressed = cp.vl['Dyn_CMM']['P002_Com_rAPP'] > 0

    # brake
    ret.brakePressed = bool(cp_cam.vl['Dat_BSI']['P013_MainBrake'])
    ret.parkingBrake = cp.vl['Dyn_EasyMove']['P337_Com_stPrkBrk'] == 1 # 0: disengaged, 1: engaged, 3: brake actuator moving

    # steering wheel
    ret.steeringAngleDeg = cp.vl['STEERING_ALT']['ANGLE'] # EPS
    ret.steeringRateDeg = cp.vl['STEERING_ALT']['RATE'] * (2 * cp.vl['STEERING_ALT']['RATE_SIGN'] - 1) # convert [0,1] to [-1,1] EPS: rot. speed * rot. sign
    ret.steeringTorque = cp.vl['STEERING']['DRIVER_TORQUE']
    ret.steeringTorqueEps = cp.vl['IS_DAT_DIRA']['EPS_TORQUE']
    ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > CarControllerParams.STEER_DRIVER_ALLOWANCE, 5)
    self.eps_active = cp.vl['IS_DAT_DIRA']['EPS_STATE_LKA'] == 3 # 0: Unauthorized, 1: Authorized, 2: Available, 3: Active, 4: Defect

    # cruise
    ret.cruiseState.speed = cp_adas.vl['HS2_DAT_MDD_CMD_452']['SPEED_SETPOINT'] * CV.KPH_TO_MS # set to 255 when ACC is off, -2 kph offset from dash speed
    ret.cruiseState.enabled = cp_adas.vl['HS2_DAT_MDD_CMD_452']['RVV_ACC_ACTIVATION_REQ'] == 1
    ret.cruiseState.available = cp_adas.vl['HS2_DYN1_MDD_ETAT_2B6']['ACC_STATUS'] > 2
    ret.cruiseState.nonAdaptive = cp_adas.vl['HS2_DAT_MDD_CMD_452']['LONGITUDINAL_REGULATION_TYPE'] != 3 # 0: None, 1: CC, 2: Limiter, 3: ACC
    ret.cruiseState.standstill = bool(cp_adas.vl['HS2_DYN_UCF_MDD_32D']['VEHICLE_STANDSTILL'])
    ret.accFaulted = cp_adas.vl['HS2_DYN_UCF_MDD_32D']['ACC_ETAT_DECEL_OR_ESP_STATUS'] == 3 # 0: Inhibited, 1: Waiting, 2: Active, 3: Fault

    # gear
    if bool(cp_cam.vl['Dat_BSI']['P103_Com_bRevGear']):
      ret.gearShifter = GearShifter.reverse
    else:
      ret.gearShifter = GearShifter.drive

    # blinkers
    blinker = cp_cam.vl['HS2_DAT7_BSI_612']['CDE_CLG_ET_HDC']
    ret.leftBlinker = blinker == 1
    ret.rightBlinker = blinker == 2

    # lock info
    ret.doorOpen = any((cp_cam.vl['Dat_BSI']['DRIVER_DOOR'], cp_cam.vl['Dat_BSI']['PASSENGER_DOOR']))
    ret.seatbeltUnlatched = cp_cam.vl['RESTRAINTS']['DRIVER_SEATBELT'] != 2
    return ret

  @staticmethod
  def get_can_parsers(CP):
    if CP.carFingerprint == CAR.PSA_PEUGEOT_308_T9:
      messages = [
        ("T9_ENGINE_DYNAMICS_208", 100),
        ("T9_STEERING_TORQUE_2F5", 100),
        ("T9_STEERING_DYNAMICS_305", 100),
        ("T9_WHEEL_SPEEDS_30D", 50),
        ("T9_ENGINE_GEAR_348", 50),
        ("T9_EASY_MOVE_3AD", 50),
        ("T9_BRAKE_DYNAMICS_3CD", 100),
        ("T9_BODY_STATUS_412", 20),
        ("T9_DRIVER_CRUISE_COMMAND_452", 20),
        ("T9_CRUISE_SETPOINT_50E", 10),
        ("T9_RESTRAINTS_572", 10),
      ]

      return {
        Bus.main: CANParser(
          DBC[CP.carFingerprint][Bus.pt],
          messages,
          0,
        ),
      }

    return {
      Bus.main: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 0),
      Bus.adas: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 1),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 2),
    }
