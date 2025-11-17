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
