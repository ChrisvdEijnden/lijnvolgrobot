"""
G1 Robot – Line-Following and Obstacle Detection System
© 2025 Chris van den Eijnden
J.C. Automotive Technologies & Cloud Development.
Some rights reserved.
"""

import math
from time import sleep
from leaphymicropython.actuators.dcmotor import DCMotors
from leaphymicropython.actuators.ssd1306 import SSD1306I2C
from leaphymicropython.sensors.linesensor import AnalogIR
from leaphymicropython.sensors.tof import TimeOfFlight


class Motor:
    """
    Controls the robot's dual DC motor system.

    This class provides simple high-level movement commands such as
    forward, backward, turning, and stopping, by controlling two
    DC motors through the DCMotors interface.
    """

    def __init__(self):
        """Initialize motor controller and retrieve individual motor channels."""
        self.motors = DCMotors()
        self.motor_a = self.motors.motor_a
        self.motor_b = self.motors.motor_b

    def forwards(self):
        """Drive both motors forward at a predefined speed."""
        self.motor_a.forward(200)
        self.motor_b.forward(200)

    def backwards(self):
        """Drive both motors backward at a predefined speed."""
        self.motor_a.backward(150)
        self.motor_b.backward(150)

    def left(self):
        """Execute a mild left turn by applying differential motor speed."""
        self.motor_a.forward(50)
        self.motor_b.forward(130)

    def right(self):
        """Execute a mild right turn by applying differential motor speed."""
        self.motor_a.forward(130)
        self.motor_b.forward(50)

    def far_left(self):
        """Execute a sharp left turn by stopping the left motor and driving the right."""
        self.motor_a.forward(0)
        self.motor_b.forward(130)

    def far_right(self):
        """Execute a sharp right turn by stopping the right motor and driving the left."""
        self.motor_a.forward(130)
        self.motor_b.forward(0)

    def stop(self):
        """Stop both motors by setting speed to zero."""
        self.motor_a.forward(0)
        self.motor_b.forward(0)


motor = Motor()


class TOF:
    """
    Handles Time-of-Flight (TOF) distance sensors for obstacle detection.

    Provides access to the raw distances, as well as higher-level
    calculations such as angular deviation and ball detection heuristics.
    """

    def __init__(self):
        """Initialize four TOF sensors, one for each channel."""
        self.tofs = [TimeOfFlight(channel=ch) for ch in range(4)]

    def tof_distances(self):
        """Return a list of measured distances from all TOF sensors."""
        return [tof.get_distance() for tof in self.tofs]

    def delta_left_right(self):
        """Return the difference between the right-side and left-side sensor distances."""
        dist = self.tof_distances()
        return dist[3] - dist[1]

    def angle_robot(self):
        """Estimate the robot's angular deviation in degrees.

        The angle is derived from the difference between left and right
        TOF sensors and calibrated for robot geometry.
        """
        return math.degrees(math.atan(self.delta_left_right() * 3 / 47))

    def is_ball(self):
        """Heuristically detect whether a ball is present in front of the robot.

        The logic uses specific offsets and thresholds based on sensor readings.
        """
        dist = self.tof_distances()
        return self.delta_left_right() / 2 - 70 - dist[2] > 60


tof = TOF()


class IR:
    """
    Interface for the robot's analog infrared line sensors.

    Responsible for reading raw analog values and converting them into
    binary black/white classifications.
    """

    def __init__(self):
        """Initialize four analog IR sensors with configurable thresholds."""
        self.irs = [
            AnalogIR("A0", 2500),
            AnalogIR("A1", 2500),
            AnalogIR("A2", 2500),
            AnalogIR("A3", 2500),
        ]

    def ir_values(self):
        """Return raw analog values for all IR sensors."""
        return [ch.get_analog_value() for ch in self.irs]

    def ir_colors(self):
        """Return black/white results for all IR sensors using calibrated thresholds."""
        colors = [ch.black_or_white() for ch in self.irs]
        return ["B" if c == "black" else "W" for c in colors]


ir = IR()


class Action:
    """
    Decision-making engine for robot movement.

    Evaluates sensor states to determine which motor action should be
    executed under the current conditions.
    """

    def __init__(self):
        """Map situation indices to corresponding motor actions."""
        self.actions = {
            0: motor.forwards,
            1: motor.far_left,
            2: motor.far_right,
            3: motor.left,
            4: motor.right,
            5: motor.backwards,
            6: motor.stop,
        }

    def situations(self):
        """
        Evaluate all movement-related sensor conditions.

        Returns a list of boolean flags representing discrete situations,
        such as line detection, edge detection, and obstacle proximity.
        """
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
        """Execute the motor action that corresponds to the first true situation."""
        for i, state in enumerate(self.situations()):
            if state:
                return self.actions[i]()
    
    def print_move(self):
        return self.move().replace("motor.", "")


action = Action()


class Screen:
    """
    Display handler responsible for showing sensor data on OLED screens.

    The system uses two SSD1306-based I2C displays for live debugging
    and monitoring of sensor output.
    """

    def __init__(self):
        """Initialize the two OLED displays on separate I2C channels."""
        self.oled = SSD1306I2C(width=128, height=32, channel=6)
        self.oled2 = SSD1306I2C(width=128, height=32, channel=7)

    def fill_screen(self):
        """Render all current TOF and IR sensor readings onto both displays."""
        self.oled.fill(0)
        self.oled2.fill(0)

        self.oled.text(f"{tof.tof_distances()}", 0, 0)
        self.oled.text(f"{tof.is_ball()}", 0, 10)
        self.oled.text(f"{tof.angle_robot():.2f} deg", 0, 20)

        self.oled2.text(f"{ir.ir_values()}", 0, 0)
        self.oled2.text(f"{ir.ir_colors()}", 0, 10)
        self.oled2.text(f"{action.print_move()}", 0, 20)

        self.oled.show()
        self.oled2.show()

    def clean(self):
        """Clear both OLED displays."""
        self.oled.fill(0)
        self.oled.show()
        self.oled2.fill(0)
        self.oled2.show()

    def terminal_prints(self):
        """Print all sensor data to the terminal for debugging."""
        print(f"TOF: {tof.tof_distances()}")
        print(f"Ball: {tof.is_ball()}")
        print(f"Angle: {tof.angle_robot():.2f}")
        print(f"IR: {ir.ir_values()}")
        print(f"Color: {ir.ir_colors()}\n")


screen = Screen()


def main():
    """
    Main execution loop for the robot.

    Continuously updates the display, prints sensor data,
    evaluates the current situation, and triggers movement logic.
    """
    while True:
        action.move()
        screen.fill_screen()
        screen.terminal_prints()
        sleep(0.01)


try:
    main()
except Exception as e:
    print(f"error: {e}")
finally:
    screen.clean()
    motor.stop()
