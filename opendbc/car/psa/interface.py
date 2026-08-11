from opendbc.car import structs, get_safety_config
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.psa.carcontroller import CarController
from opendbc.car.psa.carstate import CarState
from opendbc.car.psa.values import CAR

TransmissionType = structs.CarParams.TransmissionType


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = 'psa'

    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.psa)]

    ret.dashcamOnly = True

    ret.steerActuatorDelay = 0.3
    ret.steerLimitTimer = 0.1
    ret.steerAtStandstill = True

    ret.steerControlType = structs.CarParams.SteerControlType.angle
    ret.radarUnavailable = True

    ret.alphaLongitudinalAvailable = False

    if candidate == CAR.PSA_PEUGEOT_308_T9:
      # The T9 is deliberately observation-only. Passive evidence identifies
      # an R2 torque API, but no actuation state machine has been validated.
      ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.noOutput)]
      ret.openpilotLongitudinalControl = False
      ret.pcmCruise = True
      ret.autoResumeSng = False

    return ret
