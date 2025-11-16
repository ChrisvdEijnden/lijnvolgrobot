from leaphymicropython.actuators.dcmotor import DCMotors

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
