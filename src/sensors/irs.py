from leaphymicropython.sensors.linesensor import AnalogIR

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
