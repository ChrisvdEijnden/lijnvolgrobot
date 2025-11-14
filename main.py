"""
G1 Robot – Line-Following and Obstacle Detection System
© 2025 Chris van den Eijnden
J.C. Automotive Technologies & Cloud Development.
Some rights reserved.

This work is licensed under the Creative Commons Attribution–NonCommercial–ShareAlike 4.0 International License.
To view a copy of this license, visit https://creativecommons.org/licenses/by-nc-sa/4.0/

You are free to:
- Share — copy and redistribute the material in any medium or format.
- Adapt — remix, transform, and build upon the material.

Under the following terms:
- Attribution — You must give appropriate credit, provide a link to the license,
and indicate if changes were made.
- NonCommercial — You may not use the material for commercial purposes.
- ShareAlike — If you remix, transform, or build upon the material,
  you must distribute your contributions under the same license as the original. """

import math
from time import sleep
from leaphymicropython.actuators.dcmotor import DCMotors
from leaphymicropython.actuators.ssd1306 import SSD1306I2C
from leaphymicropython.sensors.linesensor import AnalogIR
from leaphymicropython.sensors.tof import TimeOfFlight


class Motor:
    def __init__(self):
        self.motors = DCMotors()
        self.motor_a = self.motors.motor_a
        self.motor_b = self.motors.motor_b

    def forwards(self):
        self.motor_a.forward(200)
        self.motor_b.forward(200)

    def backwards(self):
        self.motor_a.backward(150)
        self.motor_b.backward(150)

    def left(self):
        self.motor_a.forward(50)
        self.motor_b.forward(130)

    def right(self):
        self.motor_a.forward(130)
        self.motor_b.forward(50)

    def far_left(self):
        self.motor_a.forward(0)
        self.motor_b.forward(130)

    def far_right(self):
        self.motor_a.forward(130)
        self.motor_b.forward(0)

    def stop(self):
        self.motor_a.forward(0)
        self.motor_b.forward(0)


motor = Motor()


class TOF:
    def __init__(self):
        self.tofs = [TimeOfFlight(channel=ch) for ch in range(4)]

    def tof_distances(self):
        return [tof.get_distance() for tof in self.tofs]

    def delta_left_right(self):
        dist = self.tof_distances()
        return dist[3] - dist[1]

    def angle_robot(self):
        return math.degrees(math.atan(self.delta_left_right() * 3 / 47))

    def is_ball(self):
        dist = self.tof_distances()
        return self.delta_left_right() / 2 - 70 - dist[2] > 60


tof = TOF()


class IR:
    def __init__(self):
        self.irs = [
            AnalogIR("A0", 2500),
            AnalogIR("A1", 2500),
            AnalogIR("A2", 2500),
            AnalogIR("A3", 2500),
        ]

    def ir_values(self):
        return [ch.get_analog_value() for ch in self.irs]

    def ir_colors(self):
        colors = [ch.black_or_white() for ch in self.irs]
        return ["B" if c == "black" else "W" for c in colors]


ir = IR()


class Screen:
    def __init__(self):
        self.oled = SSD1306I2C(width=128, height=32, channel=6)
        self.oled2 = SSD1306I2C(width=128, height=32, channel=7)

    def fill_screen(self):
        self.oled.fill(0)
        self.oled2.fill(0)

        self.oled.text(f"{tof.tof_distances()}", 0, 0)
        self.oled.text(f"{tof.is_ball()}", 0, 10)
        self.oled.text(f"{tof.angle_robot():.2f} deg", 0, 20)

        self.oled2.text(f"{ir.ir_values()}", 0, 0)
        self.oled2.text(f"{ir.ir_colors()}", 0, 10)

        self.oled.show()
        self.oled2.show()

    def clean(self):
        self.oled.fill(0)
        self.oled.show()
        self.oled2.fill(0)
        self.oled2.show()

    def terminal_prints(self):
        print(f"TOF: {tof.tof_distances()}")
        print(f"Ball: {tof.is_ball()}")
        print(f"Angle: {tof.angle_robot():.2f}")
        print(f"IR: {ir.ir_values()}")
        print(f"Color: {ir.ir_colors()}\n")


screen = Screen()


class Action:
    def __init__(self):

        self.actions = {
            0: motor.forwards,
            1: motor.far_left,
            2: motor.left,
            3: motor.right,
            4: motor.far_right,
            5: motor.backwards,
            6: motor.stop,
        }

    def situations(self):
        colors = ir.ir_colors()
        d = tof.tof_distances()

        return [
            colors == ['W', 'W', 'W', 'W'],
            colors[0] == 'B',
            colors[3] == 'B',
            colors[1] == 'B',
            colors[2] == 'B',
            d[1] <= 40 or d[3] <= 40,
            False
        ]

    def move(self):
        for i, state in enumerate(self.situations()):
            if state:
                return self.actions[i]()


action = Action()


def main():
    while True:
        screen.fill_screen()
        screen.terminal_prints()
        action.move()
        sleep(0.1)


try:
    main()
except Exception as e:
    print(f"error: {e}")
finally:
    screen.clean()
    motor.stop()
    print("toedeloe")
