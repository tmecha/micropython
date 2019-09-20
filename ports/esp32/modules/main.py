from MotionUnit import MotionUnit
import time

motionUnit = MotionUnit(name="Xaxis")
print("Motion Unit Booted: ", motionUnit.name)
print("Serial Number: ", motionUnit.serial_number)

motionUnit.red_led_toggle()

# motionUnit.step_counter_start()

while True:
    motionUnit.grn_led_toggle()
    motionUnit.red_led_toggle()
    time.sleep(1)

    # if motionUnit.step_counter not None:
        # print("Count: ", motionUnit.step_counter.count())

    if motionUnit.user_button_pressed():
        # Step motor
        if not motionUnit.motion_move_moving:
            for i in range(10):
                print("Motor move #1")
                motionUnit.control_select_internal()
                # motionUnit.step(steps=2000, direction="Positive", frequency=200)
                motionUnit.move(direction="Positive", distance=90, accel=40000)
                while motionUnit.motion_move_moving:
                    time.sleep(1)
                motionUnit.stepper_disable()

                print("Motor move #2")
                motionUnit.move(direction="Negative", distance=30, accel=40000)
                while motionUnit.motion_move_moving:
                    time.sleep(1)
                motionUnit.stepper_disable()

                print("Motor move #3")
                motionUnit.move(direction="Negative", distance=30, accel=40000)
                while motionUnit.motion_move_moving:
                    time.sleep(1)
                motionUnit.stepper_disable()

                print("Motor move #4")
                motionUnit.move(direction="Negative", distance=30, accel=40000)
                while motionUnit.motion_move_moving:
                    time.sleep(1)
                motionUnit.stepper_disable()

    # TEST COUNTER ROLLOVER CORNER-CASES
    # if True and motionUnit.user_button_pressed():
    #     INT16_MAX = 32767
    #     INT16_MIN = -32768
    #
    #     #Set to internal control and manually set STEP pwm for testing
    #     motionUnit.control_select_internal()
    #
    #     #Step 42767 steps. Count should be 42767. Raw count should be 10,000
    #     print("--------Counter Test #1----------")
    #     motionUnit.step_counter.clear()
    #     motionUnit.step_counter.set_thresh1(42767)  # Stop Point
    #     motionUnit.step_counter.set_thresh0(100000)  # Decel point
    #     motionUnit.STEP_PWM.freq(10000)
    #     motionUnit.STEP_PWM.duty(512)  # 512=50% duty
    #     time.sleep(15)  # wait for callback
    #     print("--------Counter Test #1 Done----------")
    #
    #     print("--------Counter Test #2----------")
    #     motionUnit.step_counter.clear()
    #     print("Running Count:", motionUnit.step_counter.count(), " Raw Count:",motionUnit.step_counter.raw_reg_count())
    #     motionUnit.step_counter.set_thresh1(42767)  # Stop Point
    #     motionUnit.step_counter.set_thresh0(1000)  # Decel point
    #     motionUnit.STEP_PWM.freq(10000)
    #     motionUnit.STEP_PWM.duty(512)  # 512=50% duty
    #     time.sleep(15)  # wait for callback
    #     print("--------Counter Test #2 Done----------")
    #
    #     print("--------Counter Test #3----------")
    #     motionUnit.step_counter.clear()
    #     print("Running Count:", motionUnit.step_counter.count(), " Raw Count:",motionUnit.step_counter.raw_reg_count())
    #     motionUnit.step_counter.set_thresh1(90000)  # Stop Point
    #     motionUnit.step_counter.set_thresh0(60000)  # Decel point
    #     motionUnit.STEP_PWM.freq(10000)
    #     motionUnit.STEP_PWM.duty(512)  # 512=50% duty
    #     time.sleep(15)  # wait for callback
    #     print("--------Counter Test #3 Done----------")
    #
    #     #Step -52767. Count should be -10000
    #     #


    # print("User Button: ", motionUnit.user_button_pressed())
    # print("Home Limit: ", motionUnit.position_home.value())

    if motionUnit.check_endstops():
        print("End Stop Triggered! Stepper Disabled!")
        if motionUnit.user_button_pressed():
            print("Resetting End Stops...")
            if motionUnit.reset_endstops():
                print("End Stop Reset Success")
            else:
                print("End Stop Reset Failed")



