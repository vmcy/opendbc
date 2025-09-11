from opendbc.car.common.numpy_fast import mean, interp, clip
from opendbc.can.can_define import CANDefine
from opendbc.can.parser import CANParser
from opendbc.car import Bus, DT_CTRL, create_button_events, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.common.filter_simple import FirstOrderFilter
from opendbc.car.interfaces import CarStateBase
from opendbc.car.toyota.values import ToyotaFlags, CAR, DBC, STEER_THRESHOLD, NO_STOP_TIMER_CAR, \
                                                  TSS2_CAR, RADAR_ACC_CAR, EPS_SCALE, UNSUPPORTED_DSU_CAR
from time import time

# todo: clean this part up
pedal_counter = 0
pedal_press_state = 0
PEDAL_COUNTER_THRES = 35
PEDAL_UPPER_TRIG_THRES = 0.125
PEDAL_NON_ZERO_THRES = 0.01

SEC_HOLD_TO_STEP_SPEED = 0.6

HUD_MULTIPLIER = 1.04

ButtonType = structs.CarState.ButtonEvent.Type
SteerControlType = structs.CarParams.SteerControlType

prev_val = None

# These steering fault definitions seem to be common across LKA (torque) and LTA (angle):
# - high steer rate fault: goes to 21 or 25 for 1 frame, then 9 for 2 seconds
# - lka/lta msg drop out: goes to 9 then 11 for a combined total of 2 seconds, then 3.
#     if using the other control command, goes directly to 3 after 1.5 seconds
# - initializing: LTA can report 0 as long as STEER_TORQUE_SENSOR->STEER_ANGLE_INITIALIZING is 1,
#     and is a catch-all for LKA
TEMP_STEER_FAULTS = (0, 9, 11, 21, 25)
# - lka/lta msg drop out: 3 (recoverable)
# - prolonged high driver torque: 17 (permanent)
PERM_STEER_FAULTS = (3, 17)

class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    can_define = CANDefine(DBC[CP.carFingerprint][Bus.pt])
    self.eps_torque_scale = EPS_SCALE[CP.carFingerprint] / 100.
    self.cluster_speed_hyst_gap = CV.KPH_TO_MS / 2.
    self.cluster_min_speed = CV.KPH_TO_MS / 2.

    if CP.flags & ToyotaFlags.SECOC.value:
      self.shifter_values = can_define.dv["GEAR_PACKET_HYBRID"]["GEAR"]
    else:
      self.shifter_values = can_define.dv["TRANSMISSION"]["GEAR"]

    # On cars with cp.vl["STEER_TORQUE_SENSOR"]["STEER_ANGLE"]
    # the signal is zeroed to where the steering angle is at start.
    # Need to apply an offset as soon as the steering angle measurements are both received
    self.accurate_steer_angle_seen = False
    self.angle_offset = FirstOrderFilter(None, 60.0, DT_CTRL, initialized=False)

    self.distance_button = 0

    self.pcm_follow_distance = 0

    self.acc_type = 1
    self.lkas_hud = {}
    self.gvc = 0.0
    self.secoc_synchronization = None

    # VELOZ ADDITION
    self.shifter_values = can_define.dv["TRANSMISSION"]['GEAR']
    self.set_distance_values = can_define.dv['ACC_CMD_HUD']['FOLLOW_DISTANCE']
    self.is_cruise_latch = False
    self.cruise_speed = 30 * CV.KPH_TO_MS
    self.cruise_speed_counter = 0
    self.acttrGas = 0

    self.is_plus_btn_latch = False
    self.is_minus_btn_latch = False
    # shared by both + and - button, since release of another button will reset this
    self.rising_edge_since = 0
    self.last_frame = time()
    self.dt = 0

    self.stock_lkc_off = True
    self.stock_fcw_off = True
    self.lkas_rdy = True
    self.lkas_latch = True # Set LKAS for Perodua to True by default
    self.lkas_btn_rising_edge_seen = False
    self.stock_acc_engaged = False
    self.stock_acc_cmd = 0
    self.stock_brake_mag = 0
    self.stock_acc_set_speed = 0

  def update(self, can_parsers) -> structs.CarState:
    global prev_val
    
    cp = can_parsers[Bus.pt]

    ret = structs.CarState()

    ret.lkaDisabled = not self.lkas_latch

    ret.wheelSpeeds = self.get_wheel_speeds(
      cp.vl["WHEEL_SPEED"]['WHEELSPEED_F'],
      cp.vl["WHEEL_SPEED"]['WHEELSPEED_F'],
      cp.vl["WHEEL_SPEED"]['WHEELSPEED_F'],
      cp.vl["WHEEL_SPEED"]['WHEELSPEED_F'],
    )
    ret.vEgoRaw = float(mean([ret.wheelSpeeds.fl, ret.wheelSpeeds.fr, ret.wheelSpeeds.rl, ret.wheelSpeeds.rr]))
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.vEgoCluster = ret.vEgo * 1.015  # minimum of all the cars

    ret.standstill = abs(ret.vEgoRaw) < 1e-3

    # safety checks to engage
    can_gear = int(cp.vl["TRANSMISSION"]['GEAR'])

    ret.doorOpen = any([cp.vl["METER_CLUSTER"]['MAIN_DOOR'],
                     cp.vl["METER_CLUSTER"]['LEFT_FRONT_DOOR'],
                     cp.vl["METER_CLUSTER"]['RIGHT_BACK_DOOR'],
                     cp.vl["METER_CLUSTER"]['LEFT_BACK_DOOR']])

    ret.seatbeltUnlatched = cp.vl["METER_CLUSTER"]['SEAT_BELT_WARNING'] == 1
    ret.seatbeltUnlatched |= cp.vl["METER_CLUSTER"]['SEAT_BELT_WARNING2'] == 1
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(can_gear, None))

    self.is_cruise_latch = False if (ret.doorOpen or ret.seatbeltUnlatched) else self.is_cruise_latch

    # gas pedal
    ret.gas = cp.vl["GAS_PEDAL"]['APPS_1']
    # todo: let gas pressed legit
    ret.gasPressed = not bool(cp.vl["GAS_PEDAL_2"]['GAS_PEDAL_STEP'])

    self.acttrGas = (cp.vl["GAS_SENSOR"]['INTERCEPTOR_GAS']) # KommuActuator gas, read when stock pedal is being intercepted
    if self.acttrGas < 0:
      self.acttrGas = 0

    # brake pedal
    ret.brake = cp.vl["BRAKE"]['BRAKE_PRESSURE']

    ret.brakePressed = bool(cp.vl["BRAKE"]['BRAKE_ENGAGED'])

    # steer
    ret.steeringAngleDeg = cp.vl["STEERING_MODULE"]['STEER_ANGLE']
    ret.steeringTorque = cp.vl["STEERING_MODULE"]['MAIN_TORQUE']
    ret.steeringTorqueEps = cp.vl["EPS_SHAFT_TORQUE"]['STEERING_TORQUE']

    ret.steeringPressed = bool(abs(ret.steeringTorque) > 20)

    ret.steerWarning = False
    ret.steerError = False

    ret.vEgoCluster = cp.vl["BUTTONS"]["UI_SPEED"] * CV.KPH_TO_MS * HUD_MULTIPLIER
    ret.stockAdas.frontDepartureHUD = bool(cp.vl["LKAS_HUD"]["FRONT_DEPART"])
    ret.stockAdas.laneDepartureHUD = bool(cp.vl["LKAS_HUD"]["LDA_ALERT"])
    ret.stockAdas.ldpSteerV = cp.vl["STEERING_LKAS"]['STEER_CMD']
    ret.stockAdas.aebV = cp.vl["ACC_BRAKE"]['AEB_1019']

    ret.stockAeb = bool(cp.vl["LKAS_HUD"]['AEB_BRAKE'])
    ret.stockFcw = bool(cp.vl["LKAS_HUD"]['AEB_ALARM'])
    self.stock_lkc_off = bool(cp.vl["LKAS_HUD"]['LDA_OFF'])
    self.lkas_rdy = bool(cp.vl["LKAS_HUD"]['LKAS_SET'])
    self.stock_fcw_off = bool(cp.vl["LKAS_HUD"]['FCW_DISABLE'])

    self.stock_acc_cmd = cp.vl["ACC_CMD_HUD"]["ACC_CMD"] # kph
    self.stock_acc_engaged = self.stock_acc_cmd > 0
    self.stock_acc_set_speed = cp.vl["ACC_CMD_HUD"]["SET_SPEED"] #kph
    self.stock_brake_mag = -1 * cp.vl["ACC_BRAKE"]["MAGNITUDE"]

    # logic to engage LKC
    if bool(cp.vl["BUTTONS"]['LKC_BTN']):
      if not self.lkas_btn_rising_edge_seen:
        self.lkas_btn_rising_edge_seen = True

    if self.lkas_btn_rising_edge_seen and not bool(cp.vl["BUTTONS"]['LKC_BTN']):
      self.lkas_latch = not self.lkas_latch
      self.lkas_btn_rising_edge_seen = False

    #print(f"SET_ME_1_2: {bool(cp.vl["ACC_CMD_HUD"]["SET_ME_1_2"])}")
    #ret.cruiseState.available = bool(cp.vl["ACC_CMD_HUD"]["SET_ME_1_2"])
    ret.cruiseState.available = bool(cp.vl["PCM_BUTTONS"]["ACC_RDY"])
    distance_val = int(cp.vl["ACC_CMD_HUD"]['FOLLOW_DISTANCE'])
    #print(f"FOLLOW_DISTANCE: {int(cp.vl["ACC_CMD_HUD"]['FOLLOW_DISTANCE'])}")
    ret.cruiseState.setDistance = self.parse_set_distance(self.set_distance_values.get(distance_val, None))

    # set speed logic
    # todo: check if the logic needs to be this complicated
    minus_button = bool(cp.vl["PCM_BUTTONS"]["SET_MINUS"])
    plus_button = bool(cp.vl["PCM_BUTTONS"]["RES_PLUS"])

    if self.is_cruise_latch:
      cur_time = time()
      self.dt += cur_time - self.last_frame
      self.last_frame = cur_time

      if self.is_plus_btn_latch != plus_button: # rising or falling
        if not plus_button: # released, falling
          if cur_time - self.rising_edge_since < 1:
            self.cruise_speed += CV.KPH_TO_MS
        else: # pressed, rising, init
          self.rising_edge_since = cur_time
          self.dt = 0
      elif plus_button: # is holding
        while self.dt >= SEC_HOLD_TO_STEP_SPEED:
          kph = self.cruise_speed * CV.MS_TO_KPH
          kph += 5 - (kph % 5)  # step up to next nearest 5
          self.cruise_speed = kph * CV.KPH_TO_MS
          self.dt -= SEC_HOLD_TO_STEP_SPEED

      if self.is_minus_btn_latch != minus_button: # rising or falling
        if not minus_button: # released, falling
          if cur_time - self.rising_edge_since < 1:
            self.cruise_speed -= CV.KPH_TO_MS
        else: # pressed, rising
          self.rising_edge_since = cur_time
          self.dt = 0
      elif minus_button: # is holding
        while self.dt >= SEC_HOLD_TO_STEP_SPEED:
          kph = self.cruise_speed * CV.MS_TO_KPH
          kph = ((kph / 5) - 1) * 5  # step down to next nearest 5
          kph = max(30, kph)
          self.cruise_speed = kph * CV.KPH_TO_MS
          self.dt -= SEC_HOLD_TO_STEP_SPEED

    if not self.is_cruise_latch:
      # activate cruise onReleased
      if self.is_plus_btn_latch and not plus_button:
        self.is_cruise_latch = True

      elif self.is_minus_btn_latch and not minus_button:
        self.cruise_speed = max(30 * CV.KPH_TO_MS, ret.vEgoCluster)
        self.is_cruise_latch = True

    self.is_plus_btn_latch = plus_button
    self.is_minus_btn_latch = minus_button

    if bool(cp.vl["PCM_BUTTONS"]["CANCEL"]) or bool(cp.vl["PCM_BUTTONS_HYBRID"]["CANCEL"]) :
      self.is_cruise_latch = False

    if ret.brakePressed:
      self.is_cruise_latch = False

    # set speed in range of 30 - 140kmh only
    #print(self.stock_acc_cmd, self.stock_acc_set_speed, self.cruise_speed * 3.6)
    #self.cruise_speed = clip(self.cruise_speed, 30 * CV.KPH_TO_MS, 125 * CV.KPH_TO_MS)
    self.cruise_speed = max(min(self.cruise_speed, 140 * CV.KPH_TO_MS), 30 * CV.KPH_TO_MS)
    ret.cruiseState.speedCluster = self.cruise_speed
    ret.cruiseState.speed = ret.cruiseState.speedCluster / interp(ret.vEgo, [0,140], [1.0615,1.0170])

    ret.cruiseState.standstill = False
    ret.cruiseState.nonAdaptive = False
    ret.cruiseState.enabled = self.is_cruise_latch
    if not ret.cruiseState.available:
      self.is_cruise_latch = False

    # button presses
    ret.leftBlinker = bool(cp.vl["METER_CLUSTER"]["LEFT_SIGNAL"])
    ret.rightBlinker = bool(cp.vl["METER_CLUSTER"]["RIGHT_SIGNAL"])
    ret.genericToggle = bool(cp.vl["RIGHT_STALK"]["GENERIC_TOGGLE"])

    # blindspot sensors
    if self.CP.enableBsm:
      # used for lane change so its okay for the chime to work on both side.
      ret.leftBlindspot = bool(cp.vl["BSM"]["BSM_CHIME"])
      ret.rightBlindspot = bool(cp.vl["BSM"]["BSM_CHIME"])
    else:
      ret.leftBlindspot = False
      ret.rightBlindspot = False

    return ret

  @staticmethod
  def check_pedal_engage(gas,state):
    ''' Pedal engage logic '''
    global pedal_counter
    global pedal_press_state
    if (state == 0):
      if (gas > PEDAL_UPPER_TRIG_THRES):
        pedal_counter += 1
        if (pedal_counter == PEDAL_COUNTER_THRES):
          pedal_counter = 0
          return False
      if (pedal_counter > 2 and gas <= PEDAL_NON_ZERO_THRES):
        pedal_press_state = 1
        pedal_counter = 0
      return False
    if (state == 1):
      pedal_counter += 1
      if (pedal_counter == PEDAL_COUNTER_THRES):
        pedal_counter = 0
        pedal_press_state = 0
        return False
      if (gas > PEDAL_UPPER_TRIG_THRES):
        pedal_press_state = 2
        pedal_counter = 0
      return False
    if (state == 2):
      pedal_counter += 1
      if (pedal_counter == PEDAL_COUNTER_THRES):
        pedal_counter = 0
        pedal_press_state = 0
        return False
      if (gas <= PEDAL_NON_ZERO_THRES):
        pedal_counter = 0
        pedal_press_state = 0
        return True
    return False


  @staticmethod
  def get_can_parsers(CP):
    
    pt_messages = [
      ("WHEEL_SPEED", 0.),
      ("TRANSMISSION", 0),
      ("GAS_PEDAL", 0.),
      ("BRAKE", 0.),
      ("GAS_SENSOR", 0),
      ("RIGHT_STALK", 0),
      ("METER_CLUSTER", 0),
    ]

    pt_messages.append(("BSM", 0))
    pt_messages.append(("STEERING_MODULE", 0.))
    pt_messages.append(("EPS_SHAFT_TORQUE", 0.))
    pt_messages.append(("PCM_BUTTONS", 0))
    pt_messages.append(("PCM_BUTTONS_HYBRID", 0))
    pt_messages.append(("GAS_PEDAL_2", 0))
    pt_messages.append(("BUTTONS", 0))
    pt_messages.append(("LKAS_HUD", 0))
    pt_messages.append(("ACC_CMD_HUD", 0))
    pt_messages.append(("STEERING_LKAS", 0))
    pt_messages.append(("ACC_BRAKE", 0))
    
    # pt_messages = [
    #   # sig_name, sig_address, default
    #   ("WHEEL_SPEED", 50),
    #   ("TRANSMISSION", 30),
    #   ("GAS_PEDAL", 60),
    #   ("BRAKE", 100),
    #   ("RIGHT_STALK", 33),
    #   ("METER_CLUSTER", 0),
    #   ("BSM", 0),
    #   ("STEERING_MODULE", 100),
    #   ("EPS_SHAFT_TORQUE", 40),
    #   ("PCM_BUTTONS", 30),
    #   ("GAS_PEDAL_2", 0),
    #   ("BUTTONS", 50),
    # ]

    # pt_messages.append(("PCM_BUTTONS_HYBRID", 30))

    cam_messages = [("ACC_CMD_HUD", 0)]
    #cam_messages = []
    #cam_messages = [
      # sig_name, sig_address, default
      # ("LKAS_HUD", 20),
      # ("ACC_CMD_HUD", 20),
      # ("STEERING_LKAS", 40),
      # ("ACC_BRAKE", 20)
    #]

    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], pt_messages, 0),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], cam_messages, 2),
    }
