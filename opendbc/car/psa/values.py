from dataclasses import dataclass, field

from opendbc.car.structs import CarParams
from opendbc.car import Bus, CarSpecs, DbcDict, PlatformConfig, Platforms
from opendbc.car.lateral import AngleSteeringLimits
from opendbc.car.docs_definitions import CarDocs, CarHarness, CarParts
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, StdQueries

Ecu = CarParams.Ecu


class CarControllerParams:
  STEER_STEP = 1

  ANGLE_LIMITS: AngleSteeringLimits = AngleSteeringLimits(
    390,
    ([0., 5., 25.], [2.5, 1.5, .2]),
    ([0., 5., 25.], [5., 2., .3]),
  )
  STEER_DRIVER_ALLOWANCE = 5  # Driver intervention threshold, 0.5 Nm
  T9_STEER_DRIVER_THRESHOLD_RAW = 5

  def __init__(self, CP):
    # Required by the generic lateral-limit tests. PSA parameters are static.
    self.CP = CP


@dataclass
class PSACarDocs(CarDocs):
  package: str = "Adaptive Cruise Control (ACC) & Lane Assist"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.psa_a]))


@dataclass
class PSAPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {
    Bus.pt: 'psa_aee2010_r3',
  })


class CAR(Platforms):
  PSA_PEUGEOT_208 = PSAPlatformConfig(
    [PSACarDocs("Peugeot 208 2019-25")],
    CarSpecs(mass=1530, wheelbase=2.54, steerRatio=17.6),
  )

  # Peugeot 308 II T9 / EMP2, 2018.
  #
  # Passive captures identify the steering API as AEE2010 R2 torque. The
  # native integration remains deliberately read-only:
  # - dashcamOnly and noOutput are enforced in interface.py;
  # - CarController returns no CAN messages for this platform;
  # - the existing generic PSA R3 controller is not activated for this car.
  PSA_PEUGEOT_308_T9 = PSAPlatformConfig(
    [PSACarDocs(
      "Peugeot 308 II T9 2018",
      package="Read-only CAN; R2 torque API observed",
      car_parts=CarParts(),
    )],
    CarSpecs(
      mass=1300,
      wheelbase=2.62,
      steerRatio=15.0,
    ),
    dbc_dict={
      Bus.pt: 'peugeot_308_t9_2018',
    },
  )


FW_QUERY_CONFIG = FwQueryConfig(
  fw_version_regex=br"\d{9}",
  requests=[
    Request(
      [StdQueries.TESTER_PRESENT_REQUEST, StdQueries.UDS_VERSION_REQUEST],
      [StdQueries.TESTER_PRESENT_RESPONSE, StdQueries.UDS_VERSION_RESPONSE],
      bus=0,
    ),
  ],
)

DBC = CAR.create_dbc_map()
