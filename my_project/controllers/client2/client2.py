from controller import Robot, Motor, InertialUnit, GPS, Gyro, Keyboard, LED
import math
import socket
import time
import sys
import os
try:
    import numpy as np
except ImportError:
    sys.exit("Warning: 'numpy' module not found.")
from cryptography.fernet import Fernet

# Now you can import key_manager
sys.path.append("../..")  # Allow imports from two levels up

from key_manager import encryption_key  # Import the shared key

cipher = Fernet(encryption_key)

print(cipher)

#time.sleep(2)

# Connect to server
# def connect_server(initialization=False):
    #HOST = '127.0.0.1'  # The server's hostname or IP address
    #PORT = 5555        # The port used by the server
    
    #print("Connecting to server.")
    #client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    #client_socket.connect((HOST, PORT))
    
    # client_socket.send(b'Hello, World!')
    # data = client_socket.recv(1024)
    
    # print(f"Received: {data!r}")
    
    # print("Connection complete", client_socket)
    #return client_socket
    
    #if initialization == True:
    #    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    #       s.connect((HOST, PORT))
    #       s.sendall(b'Hello, world')
    #       data = s.recv(1024)
                
    #    print(f"Received: {data!r}")

def connect_server():
    HOST = '127.0.0.1'
    PORT = 5555
    connection_attempts = 0

    while True:
        try:
            client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client_socket.connect((HOST, PORT))
            print("✅ Connected to server.")
            return client_socket
        except ConnectionRefusedError:
            if connection_attempts <= 20:
                print("❌ Server not available. Retrying...")
                connection_attempts += 1
                continue
            else:
                break

# client_socket = connect_server(initialization=True)
client_socket = connect_server()

# Helper functions
def clamp(value, low, high):
    return max(low, min(value, high))

def sign(x):
    return (x > 0) - (x < 0)

# Initialize robot
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Get and enable devices
camera = robot.getDevice("camera")
camera.enable(timestep)
imu = robot.getDevice("inertial unit")
imu.enable(timestep)
gps = robot.getDevice("gps")
gps.enable(timestep)
compass = robot.getDevice("compass")
compass.enable(timestep)
gyro = robot.getDevice("gyro")
gyro.enable(timestep)
keyboard = Keyboard()
keyboard.enable(timestep)
front_left_led = robot.getDevice("front left led")
front_right_led = robot.getDevice("front right led")

# Camera stabilization motors
camera_roll_motor = robot.getDevice("camera roll")
camera_pitch_motor = robot.getDevice("camera pitch")

# Propeller motors
motor_names = ["front left propeller", "front right propeller", "rear left propeller", "rear right propeller"]
motors = [robot.getDevice(name) for name in motor_names]
for motor in motors:
    motor.setPosition(float('inf'))
    motor.setVelocity(1.0)

print("Start the drone...")
while robot.step(timestep) != -1:
    if robot.getTime() > 1.0:
        break

print("Drone controls:")
print("- Arrow keys: Move forward/backward/rotate")
print("- Shift + Arrow keys: Strafe/increase altitude")

# PID Constants
K_VERTICAL_THRUST = 68.5
K_VERTICAL_OFFSET = 0.6
K_VERTICAL_P = 1.5
K_HORIZONTAL_P = 0.5
K_YAW_P = 0.5
K_ROLL_P = 30.0
K_PITCH_P = 20.0

target_altitude = 1.0

def get_distance(target, current):
    """Compute Euclidean distance between target and current position."""
    return math.sqrt(sum((target[i] - current[i]) ** 2 for i in range(3)))

import math
import sys

def clamp(value, min_value, max_value):
    """Clamp value within the given range."""
    return max(min(value, max_value), min_value)

def get_distance(target, current):
    """Compute Euclidean distance between target and current position."""
    return math.sqrt(sum((target[i] - current[i]) ** 2 for i in range(3)))

def emergency_landing():
    """Moves the drone smoothly to (0,0,0.2) and lands safely without flipping."""
    print("🛑 Emergency landing initiated...")

    target_position = [0.0, 0.0, 0.2]  # Safe landing target
    descent_speed = 0.02  # Slower descent
    stabilization_factor = 0.5  # Reduce aggressive correction
    base_thrust = K_VERTICAL_THRUST  
    min_thrust = 5.5  # Prevent shutdown mid-air

    while robot.step(timestep) != -1:
        pos = gps.getValues()
        distance = get_distance(target_position, pos)

        if distance < 0.02:  # Safe landing threshold
            print("✅ Drone landed safely. Shutting down.")
            break

        # **DYNAMICALLY ADJUST ROLL/PITCH BASED ON ERROR**
        x_error = target_position[0] - pos[0]
        y_error = target_position[1] - pos[1]
        altitude_error = target_position[2] - pos[2]

        # **NEW: Scale roll/pitch correction down as it stabilizes**
        roll_input = clamp(stabilization_factor * x_error * 0.25, -0.05, 0.05)  
        pitch_input = clamp(stabilization_factor * y_error * 0.5, -0.05, 0.05)  
        vertical_input = clamp(altitude_error * 0.4, -descent_speed, descent_speed)

        print(x_error, y_error, altitude_error, roll_input, pitch_input, vertical_input)

        # **Balance thrust to counteract flipping**
        current_thrust = base_thrust + vertical_input

        motor_speeds = [
            clamp(current_thrust - roll_input + pitch_input, min_thrust, base_thrust),  # Front Left
            clamp(current_thrust + roll_input + pitch_input, min_thrust, base_thrust),  # Front Right
            clamp(current_thrust - roll_input - pitch_input, min_thrust, base_thrust),  # Rear Left
            clamp(current_thrust + roll_input - pitch_input, min_thrust, base_thrust),  # Rear Right
        ]

        # Apply new motor speeds
        for i in range(4):
            motors[i].setVelocity(motor_speeds[i])

    # Stop all motors after landing
    for motor in motors:
        motor.setVelocity(0.0)

    sys.exit("🚀 Drone has landed safely.")

while robot.step(timestep) != -1:
    time_led = robot.getTime()
    roll, pitch, _ = imu.getRollPitchYaw()
    altitude = gps.getValues()[2]
    roll_velocity, pitch_velocity, _ = gyro.getValues()

    # LED blinking
    led_state = int(time_led) % 2
    front_left_led.set(led_state)
    front_right_led.set(not led_state)

    # Camera stabilization
    camera_roll_motor.setPosition(-0.115 * roll_velocity)
    camera_pitch_motor.setPosition(-0.1 * pitch_velocity)

    # Keyboard input
    roll_disturbance, pitch_disturbance, yaw_disturbance = 0.0, 0.0, 0.0
    key = keyboard.getKey()
    while key > 0:
        if key == Keyboard.UP:
            pitch_disturbance = -2.0
        elif key == Keyboard.DOWN:
            pitch_disturbance = 2.0
        elif key == Keyboard.RIGHT:
            yaw_disturbance = -1.3
        elif key == Keyboard.LEFT:
            yaw_disturbance = 1.3
        elif key == (Keyboard.SHIFT + Keyboard.RIGHT):
            roll_disturbance = -1.0
        elif key == (Keyboard.SHIFT + Keyboard.LEFT):
            roll_disturbance = 1.0
        elif key == (Keyboard.SHIFT + Keyboard.UP):
            target_altitude += 0.05
            print(f"Target altitude: {target_altitude:.2f} m")
        elif key == (Keyboard.SHIFT + Keyboard.DOWN):
            target_altitude -= 0.05
            print(f"Target altitude: {target_altitude:.2f} m")
        key = keyboard.getKey()

    # Compute control inputs
    roll_input = K_ROLL_P * clamp(roll, -1.0, 1.0) + roll_velocity + roll_disturbance
    pitch_input = K_PITCH_P * clamp(pitch, -1.0, 1.0) + pitch_velocity + pitch_disturbance
    yaw_input = yaw_disturbance
    clamped_altitude_diff = clamp(target_altitude - altitude + K_VERTICAL_OFFSET, -1.0, 1.0)
    vertical_input = K_VERTICAL_P * (clamped_altitude_diff ** 3)

    #print(roll_input, pitch_input)

    # Compute motor velocities
    front_left_motor_input = K_VERTICAL_THRUST + vertical_input - roll_input + pitch_input - yaw_input
    front_right_motor_input = K_VERTICAL_THRUST + vertical_input + roll_input + pitch_input + yaw_input
    rear_left_motor_input = K_VERTICAL_THRUST + vertical_input - roll_input - pitch_input + yaw_input
    rear_right_motor_input = K_VERTICAL_THRUST + vertical_input + roll_input - pitch_input - yaw_input

    motors[0].setVelocity(front_left_motor_input)
    motors[1].setVelocity(-front_right_motor_input)
    motors[2].setVelocity(-rear_left_motor_input)
    motors[3].setVelocity(rear_right_motor_input)
    
    # Prepare telemetry data
    telemetry_data = f"Time: {time_led:.2f}, Altitude: {altitude:.2f}, Roll: {roll:.2f}, Pitch: {pitch:.2f}"
    #print(f"Telemetry Data: {telemetry_data}")
    encrypted_telemetry_data = cipher.encrypt(telemetry_data.encode())
    
    if client_socket:
        try:
            client_socket.send(encrypted_telemetry_data)
            #print(f"📡 Sent: {encrypted_telemetry_data}")
        except:
            print(f"⚠️ Connection lost! Reconnecting...")
            client_socket.close()
            client_socket = connect_server()

    if not client_socket:
        emergency_landing()
        break

    # try:
        #client_socket.send(encrypted_telemetry_data)
        # print(f"📡 Sent: {encrypted_telemetry_data}")
    # except (BrokenPipeError, ConnectionResetError):
    #except:
        # print(f"⚠️ Connection lost! Reconnecting...")
        # client_socket.close()

        # try:
            # client_socket = connect_server()  # Reconnect to the server
        # except:
            # emergency_landing()
        
        # pass
            
    

client_socket.close()