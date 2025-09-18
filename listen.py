import socket
import struct
import io
import threading
import time
from time import monotonic
import RPi.GPIO as GPIO
from picamera2 import Picamera2

# Network Configuration
HOST = '0.0.0.0'
WHEEL_PORT = 8000
CAMERA_PORT = 8001
PID_CONFIG_PORT = 8002

# Pins
RIGHT_MOTOR_ENA = 18
RIGHT_MOTOR_IN1 = 17
RIGHT_MOTOR_IN2 = 27
LEFT_MOTOR_ENB = 25
LEFT_MOTOR_IN3 = 23
LEFT_MOTOR_IN4 = 24
LEFT_ENCODER = 26
RIGHT_ENCODER = 16

# PID Constants (default values, can be overridden by client via socket)
use_PID = 0
KP, KI, KD = 0, 0, 0
MAX_CORRECTION = 30  # Maximum PWM correction value (works for both linear & rotational)

# Global variables
running = True
left_pwm, right_pwm = 0, 0
left_count, right_count = 0, 0
prev_left_state, prev_right_state = None, None

use_ramping = True
RAMP_RATE = 250      # PWM units per second
MIN_RAMP_THRESHOLD = 15
MIN_PWM_THRESHOLD = 15

current_movement, prev_movement = 'stop', 'stop'

def setup_gpio():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    # Motor
    GPIO.setup(RIGHT_MOTOR_ENA, GPIO.OUT)
    GPIO.setup(RIGHT_MOTOR_IN1, GPIO.OUT)
    GPIO.setup(RIGHT_MOTOR_IN2, GPIO.OUT)
    GPIO.setup(LEFT_MOTOR_ENB, GPIO.OUT)
    GPIO.setup(LEFT_MOTOR_IN3, GPIO.OUT)
    GPIO.setup(LEFT_MOTOR_IN4, GPIO.OUT)

    # Prevent jerk on startup
    GPIO.output(RIGHT_MOTOR_ENA, GPIO.LOW)
    GPIO.output(LEFT_MOTOR_ENB, GPIO.LOW)

    # Encoders
    GPIO.setup(LEFT_ENCODER, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(RIGHT_ENCODER, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.add_event_detect(LEFT_ENCODER, GPIO.BOTH, callback=left_encoder_callback)
    GPIO.add_event_detect(RIGHT_ENCODER, GPIO.BOTH, callback=right_encoder_callback)

    # PWM (100 Hz)
    global left_motor_pwm, right_motor_pwm
    left_motor_pwm = GPIO.PWM(LEFT_MOTOR_ENB, 100)
    right_motor_pwm = GPIO.PWM(RIGHT_MOTOR_ENA, 100)
    left_motor_pwm.start(0)
    right_motor_pwm.start(0)

def left_encoder_callback(channel):
    global left_count, prev_left_state
    current_state = GPIO.input(LEFT_ENCODER)
    if (prev_left_state is not None and current_state != prev_left_state):
        left_count += 1
        prev_left_state = current_state
    elif prev_left_state is None:
        prev_left_state = current_state

def right_encoder_callback(channel):
    global right_count, prev_right_state
    current_state = GPIO.input(RIGHT_ENCODER)
    if (prev_right_state is not None and current_state != prev_right_state):
        right_count += 1
        prev_right_state = current_state
    elif prev_right_state is None:
        prev_right_state = current_state

def reset_encoder():
    global left_count, right_count
    left_count, right_count = 0, 0

def set_motors(left, right):
    global prev_movement, current_movement

    # Pre-Start Kick (motor priming) to reduce jerk & bias
    if prev_movement == 'stop' and current_movement in ['forward', 'backward']:
        if current_movement == 'forward':
            GPIO.output(RIGHT_MOTOR_IN1, GPIO.HIGH)
            GPIO.output(RIGHT_MOTOR_IN2, GPIO.LOW)
            GPIO.output(LEFT_MOTOR_IN3, GPIO.HIGH)
            GPIO.output(LEFT_MOTOR_IN4, GPIO.LOW)
        elif current_movement == 'backward':
            GPIO.output(RIGHT_MOTOR_IN1, GPIO.LOW)
            GPIO.output(RIGHT_MOTOR_IN2, GPIO.HIGH)
            GPIO.output(LEFT_MOTOR_IN3, GPIO.LOW)
            GPIO.output(LEFT_MOTOR_IN4, GPIO.HIGH)
        left_motor_pwm.ChangeDutyCycle(100)
        right_motor_pwm.ChangeDutyCycle(100)
        time.sleep(0.05)

    # Right motor
    if right > 0:
        GPIO.output(RIGHT_MOTOR_IN1, GPIO.HIGH)
        GPIO.output(RIGHT_MOTOR_IN2, GPIO.LOW)
        right_motor_pwm.ChangeDutyCycle(min(right, 100))
    elif right < 0:
        GPIO.output(RIGHT_MOTOR_IN1, GPIO.LOW)
        GPIO.output(RIGHT_MOTOR_IN2, GPIO.HIGH)
        right_motor_pwm.ChangeDutyCycle(min(abs(right), 100))
    else:
        # Active braking
        GPIO.output(RIGHT_MOTOR_IN1, GPIO.HIGH)
        GPIO.output(RIGHT_MOTOR_IN2, GPIO.HIGH)
        right_motor_pwm.ChangeDutyCycle(100)

    # Left motor
    if left > 0:
        GPIO.output(LEFT_MOTOR_IN3, GPIO.HIGH)
        GPIO.output(LEFT_MOTOR_IN4, GPIO.LOW)
        left_motor_pwm.ChangeDutyCycle(min(left, 100))
    elif left < 0:
        GPIO.output(LEFT_MOTOR_IN3, GPIO.LOW)
        GPIO.output(LEFT_MOTOR_IN4, GPIO.HIGH)
        left_motor_pwm.ChangeDutyCycle(min(abs(left), 100))
    else:
        # Active braking
        GPIO.output(LEFT_MOTOR_IN3, GPIO.HIGH)
        GPIO.output(LEFT_MOTOR_IN4, GPIO.HIGH)
        left_motor_pwm.ChangeDutyCycle(100)

def apply_min_threshold(pwm_value, min_threshold):
    if pwm_value == 0:
        return 0
    elif abs(pwm_value) < min_threshold:
        return min_threshold if pwm_value > 0 else -min_threshold
    else:
        return pwm_value

def sgn(x):
    return 1 if x >= 0 else -1

def pid_control():
    global left_pwm, right_pwm, left_count, right_count
    global use_PID, KP, KI, KD, prev_movement, current_movement

    # Separate integrators for linear vs rotational
    linear_integral = 0.0
    linear_last_error = 0.0

    turn_integral = 0.0
    turn_last_error = 0.0

    last_time = monotonic()

    # Ramping state
    ramp_left_pwm = 0.0
    ramp_right_pwm = 0.0
    previous_left_target = 0.0
    previous_right_target = 0.0

    def sgn(x):
        return 1 if x >= 0 else -1

    while running:
        now = monotonic()
        dt = now - last_time
        last_time = now

        # --- Movement mode detection & reset on change ---
        prev_movement = current_movement
        if left_pwm == 0 and right_pwm == 0:
            current_movement = 'stop'
        elif left_pwm * right_pwm < 0:
            current_movement = 'turn'        # opposite signs → in-place rotation
        elif (left_pwm > 0 and right_pwm > 0):
            current_movement = 'forward'
        elif (left_pwm < 0 and right_pwm < 0):
            current_movement = 'backward'
        else:
            current_movement = 'turn'        # mixed sign with a zero → treat as turn

        if current_movement != prev_movement:
            reset_encoder()
            linear_integral = 0.0
            linear_last_error = 0.0
            turn_integral = 0.0
            turn_last_error = 0.0

        # Defaults if PID disabled
        target_left_pwm = left_pwm
        target_right_pwm = right_pwm

        # --- PID control ---
        if use_PID and (KP or KI or KD):
            if current_movement in ('forward', 'backward'):
                # Linear PID keeps robot straight by equalizing counts
                error = left_count - right_count
                proportional = KP * error
                linear_integral += KI * error * dt
                linear_integral = max(-MAX_CORRECTION, min(linear_integral, MAX_CORRECTION))
                derivative = KD * (error - linear_last_error) / dt if dt > 0 else 0.0
                correction = proportional + linear_integral + derivative
                correction = max(-MAX_CORRECTION, min(correction, MAX_CORRECTION))
                linear_last_error = error

                # Backward flips sense
                if current_movement == 'backward':
                    correction = -correction

                target_left_pwm = left_pwm - correction
                target_right_pwm = right_pwm + correction

            elif current_movement == 'turn':
                # Rotational PID balances magnitudes: |left| == |right|
                error_turn = abs(left_count) - abs(right_count)
                proportional = KP * error_turn
                turn_integral += KI * error_turn * dt
                turn_integral = max(-MAX_CORRECTION, min(turn_integral, MAX_CORRECTION))
                derivative = KD * (error_turn - turn_last_error) / dt if dt > 0 else 0.0
                correction = proportional + turn_integral + derivative
                correction = max(-MAX_CORRECTION, min(correction, MAX_CORRECTION))
                turn_last_error = error_turn

                # Adjust each wheel respecting its sign so the faster wheel slows
                target_left_pwm  = left_pwm  - sgn(left_pwm)  * correction
                target_right_pwm = right_pwm + sgn(right_pwm) * correction
            else:
                # Stop
                linear_integral = linear_last_error = 0.0
                turn_integral = turn_last_error = 0.0
                reset_encoder()
                target_left_pwm = left_pwm
                target_right_pwm = right_pwm

        # --- Ramping (shared) ---
        if use_ramping and use_PID:
            max_change = RAMP_RATE * dt

            left_diff = target_left_pwm - ramp_left_pwm
            right_diff = target_right_pwm - ramp_right_pwm

            left_needs_ramp = abs(left_diff) > MIN_RAMP_THRESHOLD
            right_needs_ramp = abs(right_diff) > MIN_RAMP_THRESHOLD

            left_dir_change = (target_left_pwm * previous_left_target < 0) and target_left_pwm != 0 and previous_left_target != 0
            right_dir_change = (target_right_pwm * previous_right_target < 0) and target_right_pwm != 0 and previous_right_target != 0

            # Immediate jump on direction changes for safety
            if left_dir_change:
                ramp_left_pwm = target_left_pwm
            if right_dir_change:
                ramp_right_pwm = target_right_pwm

            if not left_dir_change and not right_dir_change:
                if left_needs_ramp or right_needs_ramp:
                    # Left ramp
                    if abs(left_diff) <= max_change:
                        ramp_left_pwm = target_left_pwm
                    else:
                        ramp_left_pwm += max_change if left_diff > 0 else -max_change
                    # Right ramp
                    if abs(right_diff) <= max_change:
                        ramp_right_pwm = target_right_pwm
                    else:
                        ramp_right_pwm += max_change if right_diff > 0 else -max_change
                else:
                    ramp_left_pwm = target_left_pwm
                    ramp_right_pwm = target_right_pwm

            previous_left_target = target_left_pwm
            previous_right_target = target_right_pwm
        else:
            ramp_left_pwm = target_left_pwm
            ramp_right_pwm = target_right_pwm

        final_left_pwm = apply_min_threshold(ramp_left_pwm, MIN_PWM_THRESHOLD)
        final_right_pwm = apply_min_threshold(ramp_right_pwm, MIN_PWM_THRESHOLD)
        set_motors(final_left_pwm, final_right_pwm)

        # Debug
        if ramp_left_pwm != 0 or ramp_right_pwm != 0:
            print(f"[{current_movement}] (PWM L,R)=({ramp_left_pwm:.1f},{ramp_right_pwm:.1f}) "
                  f"(Enc L,R)=({left_count},{right_count})")

        time.sleep(0.01)

def camera_stream_server():
    picam2 = Picamera2()
    camera_config = picam2.create_preview_configuration(lores={"size": (640, 480)})
    picam2.configure(camera_config)
    picam2.start()

    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((HOST, CAMERA_PORT))
    server_socket.listen(1)
    print(f"Camera stream server started on port {CAMERA_PORT}")

    while running:
        try:
            client_socket, _ = server_socket.accept()
            print("Camera stream client connected")
            while running:
                stream = io.BytesIO()
                picam2.capture_file(stream, format='jpeg')
                stream.seek(0)
                jpeg_data = stream.getvalue()
                jpeg_size = len(jpeg_data)
                try:
                    client_socket.sendall(struct.pack("!I", jpeg_size))
                    client_socket.sendall(jpeg_data)
                except:
                    print("Camera stream client disconnected")
                    break
                time.sleep(0.01)
        except Exception as e:
            print(f"Camera stream server error: {str(e)}")

        if 'client_socket' in locals() and client_socket:
            client_socket.close()

    server_socket.close()
    picam2.stop()

def pid_config_server():
    global use_PID, KP, KI, KD

    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((HOST, PID_CONFIG_PORT))
    server_socket.listen(1)
    print(f"PID config server started on port {PID_CONFIG_PORT}")

    while running:
        try:
            client_socket, _ = server_socket.accept()
            print("PID config client connected")
            try:
                # Expect 4 floats: use_PID, KP, KI, KD
                data = client_socket.recv(16)
                if data and len(data) == 16:
                    use_PID, KP, KI, KD = struct.unpack("!ffff", data)
                    if use_PID:
                        print(f"Updated PID: KP={KP}, KI={KI}, KD={KD}")
                    else:
                        print("PID disabled.")
                    response = struct.pack("!i", 1)
                else:
                    response = struct.pack("!i", 0)
                client_socket.sendall(response)
            except Exception as e:
                print(f"PID config socket error: {str(e)}")
                try:
                    client_socket.sendall(struct.pack("!i", 0))
                except:
                    pass
            client_socket.close()
        except Exception as e:
            print(f"PID config server error: {str(e)}")

    server_socket.close()

def wheel_server():
    global left_pwm, right_pwm, running, left_count, right_count

    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((HOST, WHEEL_PORT))
    server_socket.listen(1)
    print(f"Wheel server started on port {WHEEL_PORT}")

    while running:
        try:
            client_socket, _ = server_socket.accept()
            print("Wheel client connected")
            while running:
                try:
                    data = client_socket.recv(8)
                    if not data or len(data) != 8:
                        print("Wheel client sending speed error")
                        break
                    left_speed, right_speed = struct.unpack("!ff", data)
                    left_pwm, right_pwm = left_speed * 100.0, right_speed * 100.0

                    # Send encoder counts
                    response = struct.pack("!ii", left_count, right_count)
                    client_socket.sendall(response)
                except Exception as e:
                    print("Wheel client disconnected")
                    break
        except Exception as e:
            print(f"Wheel server error: {str(e)}")

        if 'client_socket' in locals() and client_socket:
            client_socket.close()

    server_socket.close()

def main():
    try:
        setup_gpio()

        # PID control thread
        pid_thread = threading.Thread(target=pid_control, daemon=True)
        pid_thread.start()

        # Camera streaming thread
        camera_thread = threading.Thread(target=camera_stream_server, daemon=True)
        camera_thread.start()

        # PID configuration server thread
        pid_config_thread = threading.Thread(target=pid_config_server, daemon=True)
        pid_config_thread.start()

        # Wheel server (main thread)
        wheel_server()

    except KeyboardInterrupt:
        print("Stopping...")
    finally:
        global running
        running = False
        GPIO.cleanup()
        print("Cleanup complete")

if __name__ == "__main__":
    main()
