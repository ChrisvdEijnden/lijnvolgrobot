"""
G1 Robot – Line-Following and Obstacle Detection System
© 2025 Chris van den Eijnden
J.C. Automotive Technologies & Cloud Development.
Some rights reserved.
"""

from time import sleep
import src.sensors.tof as tof
import sensors.irs as ir
import actuators.motors as motor
from leaphymicropython.actuators.ssd1306 import SSD1306I2C


class Action:
    def __init__(self):
        self.actions = {
            0: motor.backwards,
            1: motor.forwards,
            2: motor.far_left,
            3: motor.far_right,
            4: motor.left,
            5: motor.right,
            6: motor.stop,
        }
        self.last_action = None

    def situations(self):
        colors = ir.ir_colors()
        d = tof.tof_distances()
        return [
            d[1] <= 40 or d[3] <= 40,
            colors == ['W', 'W', 'W', 'W'],
            colors[0] == 'B',
            colors[3] == 'B',
            colors[1] == 'B',
            colors[2] == 'B',
            False
        ]

    def move(self):
        """Execute the first matching action, store its name and return it."""
        for i, state in enumerate(self.situations()):
            if state:
                action_fn = self.actions[i]
                action_fn()
                self.last_action = action_fn.__name__
                return self.last_action
        return self.last_action

    def print_move(self):
        return self.last_action or "None"


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
        self.oled.fill(0)
        self.oled2.fill(0)

        self.oled.text(f"{tof.tof_distances()}", 0, 0)
        self.oled.text(f"Ball: {tof.is_ball()}", 0, 10)
        self.oled.text(f"Angle: {tof.angle():.2f}", 0, 20)

        self.oled2.text(f"{ir.ir_values()}", 0, 0)
        self.oled2.text(f"{ir.print_ir()}", 0, 10)
        self.oled2.text(f"{action.print_move()}", 0, 20)

        self.oled.show()
        self.oled2.show()

    def terminal_prints(self):
        print(f"TOF: {tof.tof_distances()}")
        print(f"Ball: {tof.is_ball()}")
        print(f"IR: {ir.ir_values()}")
        print(f"Color: {ir.print_ir()}")
        print(f"Angle: {tof.angle():.2f}")
        print(f"{action.print_move()}\n")

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
        print(f"IR: {ir.ir_values()}")
        print(f"Color: {ir.ir_colors()}")
        print(f"{action.print_move()}\n")


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
    if __name__ == "__main__":
        main()
except Exception as e:
    print(f"error: {e}")
finally:
    screen.clean()
    motor.stop()
