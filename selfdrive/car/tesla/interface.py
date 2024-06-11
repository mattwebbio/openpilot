#!/usr/bin/env python3
from cereal import car
from panda import Panda
from openpilot.common.params import Params
from openpilot.selfdrive.car.tesla.values import CAR
from openpilot.selfdrive.car import get_safety_config
from openpilot.selfdrive.car.interfaces import CarInterfaceBase


class CarInterface(CarInterfaceBase):
  @staticmethod
  def _get_params(ret, params, candidate, fingerprint, car_fw, disable_openpilot_long, experimental_long, docs):
    ret.carName = "tesla"

    # Steer blending with user torque is done virtually, and is limited to 2Nm of torque
    # before it temporarily disables OP Lat control for higher user torque. This is not
    # how openpilot typically works, hence dashcamOnly
    ret.dashcamOnly = False

    ret.steerControlType = car.CarParams.SteerControlType.angle

    # Set kP and kI to 0 over the whole speed range to have the planner accel as actuator command
    ret.longitudinalTuning.kpBP = [0]
    ret.longitudinalTuning.kpV = [0]
    ret.longitudinalTuning.kiBP = [0]
    ret.longitudinalTuning.kiV = [0]
    ret.longitudinalActuatorDelayUpperBound = 0.5 # s
    ret.radarTimeStep = (1.0 / 8) # 8Hz

    params = Params()
    stock_acc = params.get_bool("StockTaccEnabledToggle")
    if candidate in [CAR.TESLA_AP3_MODEL3, CAR.TESLA_AP3_MODELY]:
      flags = Panda.FLAG_TESLA_MODEL3_Y
      if not stock_acc:
        flags |= Panda.FLAG_TESLA_LONG_CONTROL
      ret.openpilotLongitudinalControl = not stock_acc
      ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.tesla, flags)]

    ret.steerLimitTimer = 1.0
    ret.steerActuatorDelay = 0.25
    return ret

  def _update(self, c, frogpilot_variables):
    ret, fp_ret = self.CS.update(self.cp, self.cp_cam, self.cp_adas, frogpilot_variables)

    ret.events = self.create_common_events(ret).to_msg()

    return ret, fp_ret

  def apply(self, c, now_nanos, frogpilot_variables):
    return self.CC.update(c, self.CS, now_nanos, frogpilot_variables)
