import sys
import json
from machine import Pin, PWM
import time

class Servo:
    def __init__(self, pin_number, min_pulse=500, max_pulse=2500): # Initializes the Servo with a pin number, min pulse, and max pulse duration.
        self.pwm = PWM(Pin(pin_number))
        self.pwm.freq(50)
        self.min_pulse = min_pulse
        self.max_pulse = max_pulse
        self.current_position = 0

    def degrees_to_duty(self, degrees): #Converts the degrees to a PWM duty cycle value.
        pulse_width = self.min_pulse + (degrees / 180.0) * (self.max_pulse - self.min_pulse)
        return int((pulse_width / 20000.0) * 65535)

    def set_position(self, degrees): # Sets the servo to a specific position (angle).
        if not 0 <= degrees <= 180:
            raise ValueError("Degrees must be within 0 to 180.")
        duty = self.degrees_to_duty(degrees)
        self.pwm.duty_u16(duty)
        self.current_position = degrees

    def get_position(self): # Returns the current position of the servo in degrees.
        return self.current_position

class QuadrupedController:
    def __init__(self): #  Initializes the QuadrupedController, mapping each leg's servo to its pin.
        self.servo_pins = {
            "Leg1_flex": 0,   # Leg 1 flex (ankle)
            "Leg1_side": 1,    # Leg 1 side (hip)
            "Leg2_flex": 2,    # Leg 2 flex (ankle)
            "Leg2_side": 3,    # Leg 2 side (hip)
            "Leg3_flex": 12,   # Leg 3 flex (ankle)
            "Leg3_side": 13,   # Leg 3 side (hip)
            "Leg4_flex": 14,   # Leg 4 flex (ankle)
            "Leg4_side": 15    # Leg 4 side (hip)
        }
        self.servos = {}
        self.setup_servos()

    def setup_servos(self): # Initializes all servos using the servo pins
        for servo_id, pin in self.servo_pins.items():
            self.servos[servo_id] = Servo(pin)

    def update_servos(self, positions): # Updates the positions of the servos based on the provided positions.
        for servo_id, position in positions.items():
            if servo_id in self.servos:
                self.servos[servo_id].set_position(position)

    def read_data(self): # Reads input data from input , updates the servo positions if the data is valid.
        try:
            data = sys.stdin.readline().strip()
            if data:
                positions = json.loads(data)
                self.update_servos(positions)
                return "Positions updated"
            else:
                return ""
        except Exception as e:
            return f"Error reading data: {str(e)}"


def main(): # he main loop for the quadruped controller
    controller = QuadrupedController()
    while True:
        response = controller.read_data()
        if response:
            print(response)
        time.sleep(0.1)


if __name__ == "__main__":
    main()

