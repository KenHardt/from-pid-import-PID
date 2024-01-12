import time
import RPi.GPIO as GPIO
from pid import PID

# Replace these values with the actual GPIO pin numbers you are using
PULSES_PIN = 23
DIRECTION_PIN = 22

# Replace this function with the one to control your stepper motor
def control_stepper_motor(pulses):
    # Simulate controlling the stepper motor for testing
    print(f"Moving {pulses} pulses")

def setup_gpio():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(PULSES_PIN, GPIO.OUT)
    GPIO.setup(DIRECTION_PIN, GPIO.OUT)

def cleanup_gpio():
    GPIO.cleanup()

def main():
    setup_gpio()

    # PID parameters
    Kp = 1.0  # Proportional gain
    Ki = 0.1  # Integral gain
    Kd = 0.01  # Derivative gain

    # Motor control setpoint values entered by the user
    target_velocity = float(input("Enter target velocity (pulses/second): "))
    target_distance = float(input("Enter target distance (pulses): "))

    # Conversion factor to convert distance to pulses
    pulses_per_revolution = 400
    distance_to_pulses = lambda distance: int(distance * pulses_per_revolution)

    # PID controllers for velocity and distance
    pid_velocity = PID(Kp, Ki, Kd, setpoint=target_velocity)
    pid_distance = PID(Kp, Ki, Kd, setpoint=target_distance)

    # Time interval for motor control loop
    dt = 1.0  # seconds

    total_pulses = 0

    while total_pulses < target_distance:
        # Calculate PID output for velocity
        velocity_output = pid_velocity(total_pulses)

        # Calculate PID output for distance
        distance_output = pid_distance(total_pulses)

        # Set the direction based on the sign of the distance_output
        direction_value = GPIO.HIGH if distance_output >= 0 else GPIO.LOW

        # Apply the PID outputs to control the stepper motor
        GPIO.output(DIRECTION_PIN, direction_value)
        control_stepper_motor(distance_output)

        # Print current status for monitoring
        print(f"Total Pulses: {total_pulses} | Velocity Output: {velocity_output:.2f} | Distance Output: {distance_output:.2f}")

        # Update the total pulses
        total_pulses += 1

        # Simulate pulse output (replace this with actual pulse generation code)
        GPIO.output(PULSES_PIN, GPIO.HIGH)
        time.sleep(0.001)  # Adjust this delay based on your motor's requirements
        GPIO.output(PULSES_PIN, GPIO.LOW)

        time.sleep(dt)

    print("Target distance reached. Stopping the motor.")
    cleanup_gpio()

if __name__ == "__main__":
    main()
