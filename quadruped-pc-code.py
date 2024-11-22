import serial
import json
import time
import tkinter as tk
from tkinter import ttk, messagebox

class Servo:
    """Represents a servo motor in the GUI."""
    def __init__(self, parent, servo_id, name, x, y, min_val=0, max_val=180):
        
        self.servo_id = servo_id
        self.name = name
        self.min_val = min_val
        self.max_val = max_val
        self.current_value = 90  # Default position

        self.label = tk.Label(parent, text=name)
        self.label.place(x=x, y=y)

        self.slider = tk.Scale(parent, from_=min_val, to=max_val, orient=tk.HORIZONTAL)
        self.slider.set(self.current_value)
        self.slider.place(x=x, y=y + 20)

    def get_value(self):# Returns the current value of the servo's position.
        return self.slider.get()

    def set_value(self, value): # Sets the position of the servo to a specified value.
        self.slider.set(value)


class Leg:
    """Represents a leg of the quadruped."""
    def __init__(self, parent, leg_id, base_x, base_y): # Initializes a Leg instance consisting of side and flex servos.
        self.leg_id = leg_id
        self.side_servo = Servo(parent, f"Leg{leg_id}_side", f"Leg {leg_id} Side", base_x, base_y)
        self.flex_servo = Servo(parent, f"Leg{leg_id}_flex", f"Leg {leg_id} Flex", base_x, base_y + 70)
    
    def get_positions(self): # Returns the current positions of the servos in the leg.
        return {
            f"Leg{self.leg_id}_side": self.side_servo.get_value(),
            f"Leg{self.leg_id}_flex": self.flex_servo.get_value()
        }

    def set_positions(self, side_val, flex_val): #Sets the positions of the side and flex servos for the leg.
        self.side_servo.set_value(side_val)
        self.flex_servo.set_value(flex_val)


class Quadruped:
    """Represents the entire quadruped robot."""
    def __init__(self, parent): #Initializes the Quadruped instance with 4 legs.
        self.legs = [
            Leg(parent, i + 1, x, y)
            for i, (x, y) in enumerate([(20, 20), (220, 20), (20, 200), (220, 200)])
        ]

    def get_servo_positions(self): # Retrieves the current positions of all servos across all legs.
        positions = {}
        for leg in self.legs:
            positions.update(leg.get_positions())
        return positions

    def set_positions(self, positions): #Sets the positions of all servos across all legs.
        for leg in self.legs:
            side_key = f"Leg{leg.leg_id}_side"
            flex_key = f"Leg{leg.leg_id}_flex"
            side_val = positions.get(side_key, 90)
            flex_val = positions.get(flex_key, 90)
            leg.set_positions(side_val, flex_val)


class SerialCommunicator:
    """Handles serial communication with the Pico."""
    def __init__(self, port='COM6', baud_rate=115200, timeout=1): #Initializes the serial communication with the Raspberry Pi Pico.
        try:
            self.serial_port = serial.Serial(port, baud_rate, timeout=timeout, bytesize=8, parity='N', stopbits=1)
            print(f"Connected to {port}")
        except serial.SerialException as e:
            messagebox.showerror("Serial Error", f"Could not open port {port}: {e}")
            self.serial_port = None

    def send_command(self, positions): # Sends the servo positions to the Raspberry Pi Pico over the serial connection.
        if self.serial_port and self.serial_port.is_open:
            # Convert the positions dictionary to JSON and send it over UART
            json_data = json.dumps(positions)
            self.serial_port.write(json_data.encode() + b'\n')  # Send data followed by newline

    def receive_data(self): # Reads the acknowledgment response from the Raspberry Pi Pico.
        if self.serial_port and self.serial_port.is_open:
            return self.serial_port.readline().decode().strip()  
        return ""

    def close(self): #Closes the serial port connection.
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            print("Serial port closed.")


class QuadrupedGUI:
    """Main GUI class for controlling the quadruped robot."""
    def __init__(self, root): #Initializes the GUI for controlling the quadruped.
        self.root = root
        self.quadruped = Quadruped(root)
        self.serial_comms = SerialCommunicator('COM6')

        self.create_gui()
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

    def create_gui(self):  #Creates the user interface elements.
        ttk.Button(self.root, text="Save State", command=self.save_state).place(x=400, y=20)
        ttk.Button(self.root, text="Load State", command=self.load_state).place(x=400, y=60)
        ttk.Button(self.root, text="Update Pico", command=self.update_pico).place(x=400, y=100)
        tk.Button(self.root, text="Default Pico", command=self.default_state).place(x=400, y=200)
        tk.Button(self.root, text="Stand Pico", command=self.stand_state).place(x=400, y=300)
        tk.Button(self.root, text="Wave Pico", command=self.wave).place(x=400, y=350)

    def update_pico(self): #Sends the current servo positions to the Raspberry Pi Pico.
        positions = self.quadruped.get_servo_positions()
        self.serial_comms.send_command(positions)  # Send servo positions as a JSON command
        response = self.serial_comms.receive_data()  # Wait for acknowledgment
        print("Received acknowledgment:", response)

    def save_state(self): # Saves the current servo positions to a JSON file.
        positions = self.quadruped.get_servo_positions()
        with open("quadruped_state.json", "w") as file:
            json.dump(positions, file)
        messagebox.showinfo("Success", "State saved successfully!")

    def load_state(self): #Loads the servo positions from a JSON file.
        try:
            with open("quadruped_state.json", "r") as file:
                positions = json.load(file)
            self.quadruped.set_positions(positions)
            messagebox.showinfo("Success", "State loaded successfully!")
        except FileNotFoundError:
            messagebox.showerror("Error", "No saved state found.")

    def default_state(self): # Resets the robot to the default position (90 degrees).
        default_positions = {f"Leg{i+1}_{servo}": 90 for i in range(4) for servo in ["side", "flex"]}
        self.quadruped.set_positions(default_positions)
        self.update_pico()
        messagebox.showinfo("Default State", "Robot reset to default position (90 degrees).")

    def stand_state(self): #Sets the robot to a standing position.
        standing_positions = {
            "Leg1_flex": 0,   # Leg 1 flex
            "Leg1_side": 130,   # Leg 1 side
            "Leg2_flex": 0,    # Leg 2 flex
            "Leg2_side": 45,   # Leg 2 side
            "Leg3_flex": 180,    # Leg 3 flex
            "Leg3_side": 45,    # Leg 3 side
            "Leg4_flex": 0,    # Leg 4 flex
            "Leg4_side": 130     # Leg 4 side
        }
        self.quadruped.set_positions(standing_positions)
        self.update_pico()
        messagebox.showinfo("Standing State", "Robot reset to standing position.")

    def wave(self): #Sets the robot to wave.
        wave1_positions = {
            "Leg1_flex": 180,   # Leg 1 flex
            "Leg1_side": 140,   # Leg 1 side
            "Leg2_flex": 0,    # Leg 2 flex
            "Leg2_side": 90,   # Leg 2 side
            "Leg3_flex": 180,    # Leg 3 flex
            "Leg3_side": 0,    # Leg 3 side
            "Leg4_flex": 0,    # Leg 4 flex
            "Leg4_side": 180     # Leg 4 side
        }
        wave2_positions = {
            "Leg1_flex": 180,   # Leg 1 flex
            "Leg1_side": 90,   # Leg 1 side
            "Leg2_flex": 0,    # Leg 2 flex
            "Leg2_side": 90,   # Leg 2 side
            "Leg3_flex": 180,    # Leg 3 flex
            "Leg3_side": 0,    # Leg 3 side
            "Leg4_flex": 0,    # Leg 4 flex
            "Leg4_side": 180     # Leg 4 side
        }
        for i in range(3):
            self.quadruped.set_positions(wave1_positions)
            self.update_pico()
            time.sleep(1)
            self.quadruped.set_positions(wave2_positions)
            self.update_pico()
        

        

    def on_closing(self):  #Handles the window closing event.
        self.serial_comms.close()
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    root.geometry("600x400")
    app = QuadrupedGUI(root)
    root.mainloop()
