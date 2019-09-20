from machine import Pin, Signal
from machine import PWM
from machine import Timer
from machine import I2C
from machine import unique_id
from machine import DEC
import utime

import MoveTrapezoid


class MotionUnit:
    """
    A motion unit hardware
    """
    travel_mm_rev = 4                           # millimeters of travel per revolution
    travel_microstep = 4                        # microstep setting
    travel_steps_rev = 200 * travel_microstep   # steps per revolution
    max_speed_def_mm_min = 1200                 # default maximum speed in mm per minute
    min_speed_def_freq = 200                    # minimum speed of motor, when reaching a stop

    motion_timer = None                 # timer object for motion
    motion_move_max_speed_freq = 0      # frequency of maximum speed, do not exceed
    motion_move_callback_ms = 10        # callback interval from motion acceleration function
    motion_move_freq_step = 0           # callback frequency step
    motion_move_decel_point_steps = 0   # Steps which deceleration should start
    motion_move_start_time = None       # time move started
    motion_move_moving = False          # True when moving
    motion_move_complete_flag = False   # Flag the move is complete
    motion_move_max_speed_flag = False  # Reached max speed
    motion_move_min_speed_flag = False  # Reached min speed

    flag_endstop_triggered = False
    user_btn_last_press = False  # todo: implement button debounce

    def __init__(self, name=None):
        self.name = name
        self.serial_number = unique_id()

        # Configure User Interface Pins
        self.red_led = Signal(Pin(32, Pin.OUT, value=1), invert=True)  # Note: "on" is logical pin high, LED is off
        self.grn_led = Signal(Pin(15, Pin.OUT, value=1), invert=True)  # Note: "on" is logical pin high, LED is off
        self.user_btn = Pin(36, Pin.IN)
        self.user_btn_last_press = utime.ticks_ms()  # Create an initial value for btn debounce

        # External Control Signals (EN, STEP, DIR)
        self.EN_ext = Pin(39, Pin.IN)
        self.STEP_ext = Pin(34, Pin.IN)
        self.DIR_ext = Pin(35, Pin.IN)

        # STEP feedback counter
        self.STEP_feedback = Pin(26, Pin.IN)  # This pin feeds back the output signal to the driver
        self.step_counter = None

        # Stepper Control Output Signals (EN, STEP, DIR)
        self.control_internal_sel = Signal(Pin(2, Pin.OUT, value=0))  # 0/Low = External Control, 1/High = Internal ESP32 Control
        self.stepper_control = "External"
        self.EN = Signal(Pin(13, Pin.OUT, value=0))
        self.DIR = Signal(Pin(25, Pin.OUT, value=1))
        self.STEP = Pin(12, Pin.OUT, value=0)
        self.STEP_PWM = PWM(self.STEP, freq=10, duty=0)  # duty 0 = pause, 512=50%

        # Stepper Driver Signals
        self.fault = Pin(33, Pin.IN)
        self.dmode0 = Pin(14, Pin.OUT, value=0)  # Default to 1/4 step. dmode2 fixed high.
        self.dmode1 = Pin(27, Pin.OUT, value=1)  # Default to 1/4 step. dmode2 fixed high.

        # Position Sensors
        self.position_home = Pin(18, Pin.IN, pull=Pin.PULL_UP)
        self.position_1 = Pin(16, Pin.IN, pull=Pin.PULL_UP)
        self.position_2 = Pin(23, Pin.IN, pull=Pin.PULL_UP)
        self.position_3 = Pin(17, Pin.IN, pull=Pin.PULL_UP)
        self.position_end = Pin(19, Pin.IN, pull=Pin.PULL_UP)

        # Configure Endstop Interrupts
        self.position_home.irq(trigger=Pin.IRQ_FALLING, handler=self.endstop_interrupt_handler)
        self.position_end.irq(trigger=Pin.IRQ_FALLING, handler=self.endstop_interrupt_handler)

        # Initialize I2C port
        self.i2c = I2C(0, scl=Pin(22), sda=Pin(21), freq=400000)  # AT88SC0104 EEPROM can be up to 1mhz
        print("Beginning I2C Bus Scan")
        scan = self.i2c.scan()
        if scan == []:
            print("I2C Scan Failed: No I2C device on the bus")
        else:
            print(scan)

        # Initialize step limit counter
        self.step_limit_counter_init()

    def endstop_interrupt_handler(self, pin):
        """
        Handle an interrupt generated when an endstop is hit
        :param pin:
        :return:
        """
        # Immediately Disable Stepper Motor
        self.control_internal_sel.on()
        self.EN.off()

        # trigger a flag
        self.flag_endstop_triggered = True

    def counter_test_callback(self, counter):
        event = counter.get_irq_event()
        event_flag = event.get_event()
        if event_flag & DEC.EVT_THRES_1:
            # Stop Motion
            self.STEP_PWM.duty(0)
            print("Thresh1, Stopped STEP PWM:", self.step_counter.count())
        elif event_flag & DEC.EVT_THRES_0:
            # Stop Motion
            self.STEP_PWM.duty(0)
            print("Thresh0, Stopped STEP PWM:", self.step_counter.count())

    def counter_event_interrupt_handler(self, counter):
        """
        Handle counter event interrupts
        :return:
        """
        event = counter.get_irq_event()
        if event is None:
            print("Received \'None\' Event")
            return
        event_flag = event.get_event()
        if event_flag & DEC.EVT_THRES_1:
            # Stop Motion
            self.STEP_PWM.duty(0)
            if self.motion_timer is not None:
                self.motion_timer.deinit()
            # todo: move_complete() callback
            self.motion_move_moving = False
            self.motion_move_complete_flag = True
            self.motion_move_max_speed_flag = False

            print("Stopped STEP PWM:", self.step_counter.count())
            # print("Event Count: ", event.get_event_count())

        elif event_flag & DEC.EVT_THRES_0:
            # Begin Deceleration
            self.motion_move_freq_step = -self.motion_move_freq_step
            print("Begin Decel:", self.step_counter.count())

        # print("Counter Interrupt Triggered!")
        # if event_flag & DEC.EVT_THRES_0:
        #     print("THRESH_0 Event")
        # if event_flag & DEC.EVT_THRES_1:
        #     print("THRESH_1 Event")
        # if event_flag & DEC.EVT_ZERO:
        #     print("ZERO Event")
        if event_flag & DEC.EVT_L_LIM:
            print("L_LIM Event")
        if event_flag & DEC.EVT_H_LIM:
            print("H_LIM Event")
            print("Count:",self.step_counter.count(), "evt Cnt:", event.get_event_count(), "raw cnt:",self.step_counter.raw_reg_count())

    def step_limit_counter_init(self):
        """
        Test Counter
        :return:
        """
        self.step_counter = DEC(0, self.STEP_feedback)
        self.step_counter.irq(handler=self.counter_event_interrupt_handler,
                              trigger=DEC.EVT_L_LIM | DEC.EVT_H_LIM | DEC.EVT_THRES_0 | DEC.EVT_THRES_1)

    def check_endstops(self):
        """
        Checks if the end stop limits have been exceeded
        :return:
        """
        return self.flag_endstop_triggered

    def reset_endstops(self):
        """
        Clears endstop flags. End-stops should also be physically cleared.
        End Stops are active/flagged low
        :return: True for success. False for fail.
        """
        error = False
        if self.position_home.value() == 0:
            print("Error: HOME endstop not clear! Cannot reset endstops.")
            error = True
        if self.position_end.value() == 0:
            print("Error: END endstop not clear! Cannot reset endstops.")
            error = True

        if error:
            return False
        else:
            self.flag_endstop_triggered = False
            return True

    def grn_led_toggle(self):
        """
        Toggles LED
        :return:
        """
        if self.grn_led.value():
            self.grn_led.off()
        else:
            self.grn_led.on()

    def red_led_toggle(self):
        """
        Toggles LED
        :return:
        """
        if self.red_led.value():
            self.red_led.off()
        else:
            self.red_led.on()

    def user_button_pressed(self):
        """
        Returns the value of the User Button
        :return:
        """
        if self.user_btn.value():
            return False
        else:
            return True

    def control_select_internal(self):
        """
        Changes stepper control to internal processor
        :return:
        """
        self.control_internal_sel.on()
        self.stepper_control = "Internal"

    def control_select_external(self):
        """
        Changes stepper control to external connector
        :return:
        """
        self.control_internal_sel.off()
        self.stepper_control = "External"

    def stepper_enable(self):
        """
        Enables stepper motor from internal control
        :return:
        """
        if self.stepper_control != "Internal":
            raise ValueError("Cant enable int stepper while ctrl=external")
        self.EN.on()

    def stepper_disable(self):
        """
        Disables stepper motor from internal control
        :return:
        """
        if self.stepper_control != "Internal":
            self.control_select_internal()
        self.EN.off()

    def direction_positive(self):
        """
        Sets stepper in positive direction
        :return:
        """
        self.DIR.off()

    def direction_negative(self):
        """
        Sets stepper in negative direction
        :return:
        """
        self.DIR.on()

    def direction_reverse(self):
        """
        Reverses stepper direction
        :return:
        """
        if self.DIR.value() == 0:
            self.DIR.value(1)
        else:
            self.DIR.value(0)

    def step(self, steps=None, direction="Positive", frequency=None):
        """
        Steps unit in direction, either a preset number of steps or distance
        :param steps:
        :param direction: Direction
        :param frequency:
        :return:
        """
        if direction == "Positive":
            self.direction_positive()
        elif direction == "Negative":
            self.direction_negative()
        else:
            raise ValueError("Direction Param Not Recognized")

        # Start Step Counter
        self.step_counter.clear()
        if self.motion_move_decel_point_steps:
            self.step_counter.set_thresh0(self.motion_move_decel_point_steps)  # Decel Point
        self.step_counter.set_thresh1(steps)  # Stop point

        # Enable Stepper Driver
        if not self.EN.value():
            self.stepper_enable()

        # Set moving variables
        self.motion_move_complete_flag = False
        self.motion_move_moving = True
        self.motion_move_start_time = utime.ticks_ms()

        # Start Pulsing
        self.STEP_PWM.freq(frequency)
        self.STEP_PWM.duty(512)  # 512=50% duty

        # start time for motion
        self.motion_move_start_time = utime.ticks_ms()

    def speed_to_freq(self, speed_mm_min):
        """
        Converts a speed in mm/min to pwm frequency
        I've double checked this
        :param speed_mm_min: speed in millimeters per minute
        :return: frequency in hertz
        """
        steps_per_mm = self.travel_steps_rev / self.travel_mm_rev
        frequency = int(steps_per_mm * speed_mm_min / 60)
        return frequency

    def dist_to_steps(self, distance, units='mm'):
        if units == 'mm':
            steps = int(distance * self.travel_steps_rev / self.travel_mm_rev)
        else:
            raise ValueError("Units unsupported")
        return steps

    def accel_callback(self, tim):
        # todo: write function
        # on acceleration finished, call tim.deinit()

        # increase frequency by frequency step
        current_freq = self.STEP_PWM.freq()
        if current_freq + self.motion_move_freq_step >= self.motion_move_max_speed_freq:
            # Limit to max speed
            self.STEP_PWM.freq(self.motion_move_max_speed_freq)
            if not self.motion_move_max_speed_flag:
                counter = self.step_counter.count()
                self.motion_move_max_speed_flag = True
                print("TopSpeed ms: ", utime.ticks_diff(utime.ticks_ms(), self.motion_move_start_time), " Steps:", counter)
        elif current_freq + self.motion_move_freq_step <= 0:
            # Set to minimum velocity
            self.STEP_PWM.freq(self.min_speed_def_freq)
            if not self.motion_move_min_speed_flag:
                counter = self.step_counter.count()
                self.motion_move_min_speed_flag = True
                print("reached min velo:", counter)
        else:
            # change speed
            self.STEP_PWM.freq(current_freq + self.motion_move_freq_step)

    def move(self, *, direction, distance, accel=0, max_speed=None):
        # todo: create a Motion class
        # units: distance:mm speed: mm/min accel: mm/min/min

        if self.check_endstops():
            raise ValueError("Endstop is triggered! Aborting move")

        if accel == 0:
            raise ValueError("Accel must be > 0")

        if max_speed is None:
            max_speed = self.max_speed_def_mm_min

        # calculate total steps required
        steps = self.dist_to_steps(distance, units='mm')
        print("Move total steps:", steps)

        # calculate time and distance to max speed
        accel_max_speed_dist, speed_calc, max_speed_intervals = \
            MoveTrapezoid.integrate_second_discrete(end_vel=max_speed, time_period=self.motion_move_callback_ms/60000,
                                                    time_units='min', func=MoveTrapezoid.linear_accel,
                                                    func_args={'accel': accel})

        print("Max Speed Dist mm:", accel_max_speed_dist, "Max Speed ms:", max_speed_intervals*self.motion_move_callback_ms)

        # Calculate decel_point
        self.motion_move_decel_point_steps = steps - self.dist_to_steps(accel_max_speed_dist, units='mm')

        # calculate accel rates
        self.motion_move_max_speed_freq = self.speed_to_freq(max_speed)
        self.motion_move_freq_step = int(self.motion_move_max_speed_freq / max_speed_intervals)

        if accel_max_speed_dist >= distance/2:
            # Using pyramid velocity profile
            print("Using pyramid profile, max speed not in move")
            self.motion_move_decel_point_steps = int(steps / 2)

        print("Max Speed Freq:", self.motion_move_max_speed_freq, "Freq Step:", self.motion_move_freq_step)
        if self.motion_move_freq_step == 0:
            raise ValueError("Acceleration too low, zero freq step:", accel)

        # Create acceleration timer
        if self.motion_timer is None:
            self.motion_timer = Timer(-1)

        self.step(steps=steps, direction=direction, frequency=self.speed_to_freq(self.motion_move_freq_step))  #initial speed is first accel step

        self.motion_timer.init(period=self.motion_move_callback_ms, mode=Timer.PERIODIC,
                                   callback=self.accel_callback)
