from opendbc.car import Bus, structs, get_safety_config, uds, STD_CARGO_KG, scale_rot_inertia, scale_tire_stiffness
from opendbc.car.toyota.carstate import CarState
from opendbc.car.toyota.carcontroller import CarController
from opendbc.car.toyota.radar_interface import RadarInterface
from opendbc.car.toyota.values import Ecu, CAR, DBC, ToyotaFlags, CarControllerParams, TSS2_CAR, RADAR_ACC_CAR, NO_DSU_CAR, \
                                                  MIN_ACC_SPEED, EPS_SCALE, UNSUPPORTED_DSU_CAR, NO_STOP_TIMER_CAR, ANGLE_CONTROL_CAR, \
                                                  ToyotaSafetyFlags
from opendbc.car.disable_ecu import disable_ecu
from opendbc.car.interfaces import CarInterfaceBase

SteerControlType = structs.CarParams.SteerControlType


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController
  RadarInterface = RadarInterface

  @staticmethod
  def get_pid_accel_limits(CP, current_speed, cruise_speed):
    return CarControllerParams(CP).ACCEL_MIN, CarControllerParams(CP).ACCEL_MAX

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, experimental_long, docs) -> structs.CarParams:
    ret.brand = "toyota"
    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.perodua)]
    ret.safetyConfigs[0].safetyParam = 1
    ret.transmissionType = structs.CarParams.TransmissionType.automatic
    ret.radarUnavailable = True
    ret.enableDsu = False                  # driving support unit

    ret.steerRateCost = 0.7                # Lateral MPC cost on steering rate, higher value = sharper turn
    ret.steerLimitTimer = 0.1              # time before steerLimitAlert is issued
    ret.steerControlType = structs.CarParams.SteerControlType.torque
    ret.steerActuatorDelay = 0.48          # Steering wheel actuator delay in seconds

    ret.lateralTuning.init('pid')
    ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[0.], [0.]]
    ret.longitudinalTuning.kpV = [0.9, 0.8, 0.8]

    ret.enableGasInterceptor = 0x201 in fingerprint[0] or 0x401 in fingerprint[0]
    ret.openpilotLongitudinalControl = True

    if candidate == CAR.TOYOTA_VELOZ_MY_2022:
      ret.wheelbase = 2.750
      ret.steerRatio = 17.00
      ret.centerToFront = ret.wheelbase * 0.44
      tire_stiffness_factor = 0.9871
      ret.mass = 1170. + STD_CARGO_KG
      ret.wheelSpeedFactor = 1.425

      ret.lateralTuning.pid.kiV, ret.lateralTuning.pid.kpV = [[0.16], [0.30]]
      ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0.], [255]]
      ret.lateralTuning.pid.kf = 0.00015

      ret.longitudinalTuning.kpBP = [0., 5., 20.]
      ret.longitudinalTuning.kpV = [0.15, 0.6, 0.7]
      ret.longitudinalTuning.kiBP = [5, 7, 28]
      ret.longitudinalTuning.kiV = [0.15, 0.26, 0.26]
      ret.longitudinalActuatorDelayLowerBound = 0.42
      ret.longitudinalActuatorDelayUpperBound = 0.60
      ret.speedControlled = True

      ret.minEnableSpeed = -1
      ret.steerActuatorDelay = 0.30           # Steering wheel actuator delay in seconds
      ret.enableBsm = True
      ret.stoppingDecelRate = 0.25 # reach stopping target smoothly

    ret.rotationalInertia = scale_rot_inertia(ret.mass, ret.wheelbase)
    ret.tireStiffnessFront, ret.tireStiffnessRear = scale_tire_stiffness(ret.mass, ret.wheelbase, ret.centerToFront, tire_stiffness_factor=tire_stiffness_factor)

    return ret

  # returns a car.CarState
  def update(self, can_strings, c):
    # to receive CAN Messages
    self.cp.update_strings(can_strings)

    ret = self.CS.update(self.cp)
    ret.canValid = self.cp.can_valid
    ret.steeringRateLimited = self.CC.steer_rate_limited if self.CC is not None else False
    ret.steeringRateLimited &= self.CS.lkas_rdy

    # events
    #events = self.create_common_events(ret)
    #ret.events = events.to_msg()

    self.CS.out = ret.as_reader()
    return self.CS.out, []

  # pass in a car.CarControl to be called at 100hz
  def apply(self, c):

    isLdw = c.hudControl.leftLaneDepart or c.hudControl.rightLaneDepart

    can_sends = self.CC.update(c.enabled, self.CS, self.frame, c.actuators, c.hudControl.leadVisible, c.hudControl.rightLaneVisible, c.hudControl.leftLaneVisible, c.cruiseControl.cancel, isLdw)

    self.frame += 1
    return can_sends
  
  @staticmethod
  def init(CP, can_recv, can_send):
    # disable radar if alpha longitudinal toggled on radar-ACC car
    if CP.flags & ToyotaFlags.DISABLE_RADAR.value:
      communication_control = bytes([uds.SERVICE_TYPE.COMMUNICATION_CONTROL, uds.CONTROL_TYPE.ENABLE_RX_DISABLE_TX, uds.MESSAGE_TYPE.NORMAL])
      disable_ecu(can_recv, can_send, bus=0, addr=0x750, sub_addr=0xf, com_cont_req=communication_control)
