"""
G1 Robot – Line-Following and Obstacle Detection System
© 2026 Chris van den Eijnden
J.C. Automotive Technologies & Cloud Development.
Some rights reserved.
"""

from time import sleep
from leaphymicropython.sensors.linesensor import AnalogIR
from leaphymicropython.sensors.tof import TimeOfFlight
from leaphymicropython.actuators.oled_screen import OLEDSH1106
from leaphymicropython.actuators.dcmotor import DCMotors


class Motor:
    def __init__(self):
        self.motors = DCMotors()
        self.motor_a = self.motors.motor_a
        self.motor_b = self.motors.motor_b

    def forwards(self):
        self.motor_a.forward(250)
        self.motor_b.forward(250)

    def backwards(self):
        self.motor_a.backward(200)
        self.motor_b.backward(200)

    def left(self):
        self.motor_a.forward(200)
        self.motor_b.forward(250)

    def right(self):
        self.motor_a.forward(250)
        self.motor_b.forward(200)

    def far_left(self):
        self.motor_a.forward(0)
        self.motor_b.forward(200)

    def far_right(self):
        self.motor_a.forward(200)
        self.motor_b.forward(0)

    def stop(self):
        self.motor_a.forward(0)
        self.motor_b.forward(0)

    def search(self):
        self.motor_a.forward(200)
        self.motor_b.backward(200)


motor = Motor()
        

class ToF:
    def __init__(self):
        self.tofs = [TimeOfFlight(channel=ch) for ch in range(4)]
    
    def tof_distances(self):
        return [tof.get_distance() for tof in self.tofs]

    def delta_left_right(self):
        dist = self.tof_distances()
        return (dist[3] - dist[1])/2
    
    def tof_top(self):
        diff = self.delta_left_right()
        return diff + 80 # mm
    
    def is_ball(self):
        dist = self.tof_distances()
        tof_top = int(self.tof_top())
        return tof_top - dist[2] > 70
    
    def is_obstacle(self):
        dist = self.tof_distances()
        return dist[3] - dist[2] + 80 > 80 and dist[1] - dist[2] + 80 > 80
    

tof = ToF()


class IR:
    def __init__(self):
        self.irs = [
            AnalogIR("A0", 10000), # Far left
            AnalogIR("A1", 10000), # Left
            AnalogIR("A2", 10000), # Right
            AnalogIR("A3", 10000), # Far right
        ]

    def ir_values(self):
        irs = self.irs
        return [ch.get_analog_value() for ch in irs]

    def ir_colors(self):
        irs = self.irs
        colors = [ch.black_or_white() for ch in irs]
        return ["B" if c == "black" else "W" for c in colors]
        

ir = IR()


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
            7: motor.search
        }
        self.last_action = None

    def situations(self):
        colors = ir.ir_colors()
        d = tof.tof_distances()
        return [
            d[1] <= 40 or d[3] <= 40, # backwards
            colors == ['W', 'W', 'W', 'W'], # forwards
            colors[0] == 'B', # far left
            colors[3] == 'B', # far right
            colors[1] == 'B', # left
            colors[2] == 'B', # right
            False,
            not tof.is_ball() # search
        ]

    def move(self):
        for i, state in enumerate(self.situations()):
            if state:
                action = self.actions[i]
                action()
                self.last_action = action.__name__
                return self.last_action
        return self.last_action

    def print_move(self):
        return self.last_action or "None"


action = Action()


class Screen:
    def __init__(self):
        self.oled = OLEDSH1106(width=128, height=64, channel=6)
        self.oled2 = OLEDSH1106(width=128, height=64, channel=7)
    
    def update_data(self):
        """Fetch fresh data from sensors"""
        self.dist = tof.tof_distances()
        self.col = ir.ir_colors()
        self.val = ir.ir_values()
        self.move = action.print_move()
        self.message = "Ball Found" if tof.is_ball() else "Object Found" if tof.is_obstacle() else ""
        
    def fill_screen(self):
        self.update_data()
        
        self.oled.fill('white')
        self.oled2.fill('white')

        self.oled.text(f"ToF Data:", 0, 2)
        self.oled.text(f"S:{self.dist[0]} R:{self.dist[1]}", 0, 12)
        self.oled.text(f"B:{self.dist[2]} L:{self.dist[3]}", 0, 22)
        self.oled.text(f"{self.move}", 0, 37)
        self.oled.text(f"{self.message}", 0, 47)
        
        self.oled2.text(f"IR Data:", 0, 2)
        self.oled2.text(f"FL:{self.col[0]} {self.val[0]}", 0, 12)
        self.oled2.text(f"L:{self.col[1]} {self.val[1]}", 0, 22)
        self.oled2.text(f"R:{self.col[2]} {self.val[2]}", 0, 32)
        self.oled2.text(f"FR:{self.col[3]} {self.val[3]}", 0, 42)

        self.oled.show()
        self.oled2.show()
    
    def fill_terminal(self):
        self.update_data()
        
        print(f"ToF Data:")
        print(f"Side:{self.dist[0]} Right:{self.dist[1]}")
        print(f"Bottom:{self.dist[2]} Left:{self.dist[3]}\n")
        
        print(f"Next Move: {self.move}")
        print(f"{self.message}\n")
        
        print(f"IR Data:")
        print(f"Far Left:{self.col[0]}, {self.val[0]}")
        print(f"Left:{self.col[1]}, {self.val[1]}")
        print(f"Right:{self.col[2]}, {self.val[2]}")
        print(f"Far Right:{self.col[3]}, {self.val[3]}")        
    
    def clear_screen(self):
        self.oled.fill('black')
        self.oled.show()

        self.oled2.fill('black')
        self.oled2.show()


screen = Screen()


def main():
    while True:
        action.move()
        screen.fill_screen()
        sleep(0.2)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        screen.clear_screen()
        motor.stop()
        print("Stopped")
