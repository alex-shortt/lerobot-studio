from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus
import time
import sys
import cv2
import threading
import numpy as np

# Port configuration - adjust to match your setup
FOLLOWER_PORT = "/dev/tty.usbmodem5AE60805191"

# Motor configuration
MOTOR_IDS = [1, 2, 3, 4, 5, 6]
MOTOR_MODEL = "sts3215"

# Camera configuration
CAMERA_INDEX = 1
CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 720

# =============================================================================
# FACE TRACKING TUNING - Adjust these!
# =============================================================================

# Which joints to use for tracking (since camera is on gripper, sideways)
# Try different combinations to see what works for your setup
# Motor 1 = shoulder_pan (base rotation)
# Motor 2 = shoulder_lift
# Motor 3 = elbow_flex
# Motor 4 = wrist_flex
# Motor 5 = wrist_roll

# For camera horizontal movement (face left/right in image)
HORIZONTAL_MOTOR = 1  # Which motor controls left/right in camera view
HORIZONTAL_SIGN = 1  # Flip to 1 or -1 if direction is wrong

# For camera vertical movement (face up/down in image)
VERTICAL_MOTOR = 2  # Which motor controls up/down in camera view
VERTICAL_SIGN = 1  # Flip to 1 or -1 if direction is wrong

# Speed and responsiveness
MAX_SPEED_DEG_PER_SEC = 20.0  # Maximum joint speed (degrees per second)
GAIN = 0.03  # How aggressive to track (degrees per pixel of offset)
DEADZONE_PIXELS = 50  # How far off-center before we start moving

# Smoothing to reduce jitter (0 = no smoothing, 0.9 = very smooth but laggy)
FACE_SMOOTHING = 0.7  # Smooth the detected face position
OUTPUT_SMOOTHING = 0.5  # Smooth the motor commands

# Joint limits (degrees) - widened based on your starting positions
JOINT_LIMITS = {
    1: (-180, 180),  # shoulder_pan
    2: (-180, 180),  # shoulder_lift
    3: (-180, 180),  # elbow_flex
    4: (-180, 180),  # wrist_flex
    5: (-180, 180),  # wrist_roll
    6: (-10, 100),  # gripper
}

# Threading for camera capture
stop_camera = threading.Event()
camera_thread = None
latest_frame = None
frame_lock = threading.Lock()


def make_bus(port, prefix):
    motors = {
        f"{prefix}_{i}": Motor(id=i, model=MOTOR_MODEL, norm_mode=MotorNormMode.DEGREES)
        for i in MOTOR_IDS
    }
    return FeetechMotorsBus(port=port, motors=motors)


def connect_bus(bus, name, handshake=True):
    """Connect to a motor bus with error handling."""
    try:
        print(f"Connecting to {name}...")
        bus.connect(handshake=handshake)
        print(f"✓ {name} connected successfully!")
        return True
    except RuntimeError as e:
        if "motor check failed" in str(e) or "Missing motor" in str(e):
            print(f"⚠ Warning: {name} connection failed - motors not found on port")
            print(f"  Error: {str(e).split(chr(10))[0]}")
            return False
        else:
            raise


def camera_capture_loop(camera_index, stop_event):
    """Capture frames in a background thread."""
    global latest_frame
    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        print("⚠ Warning: Could not open camera in thread.")
        return

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)

    while not stop_event.is_set():
        ret, frame = cap.read()
        if ret:
            with frame_lock:
                latest_frame = frame

    cap.release()


def detect_faces(frame, face_cascade):
    """Detect faces in a frame using OpenCV Haar cascade."""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    scale = 0.5
    small_gray = cv2.resize(gray, (0, 0), fx=scale, fy=scale)

    faces = face_cascade.detectMultiScale(
        small_gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30)
    )

    face_locations = []
    for x, y, w, h in faces:
        top = int(y / scale)
        right = int((x + w) / scale)
        bottom = int((y + h) / scale)
        left = int(x / scale)
        face_locations.append((top, right, bottom, left))

    return face_locations


def get_face_center(face_location):
    """Get the center point of a face bounding box."""
    top, right, bottom, left = face_location
    center_x = (left + right) // 2
    center_y = (top + bottom) // 2
    return center_x, center_y


def draw_overlay(frame, face_locations, frame_center, status_text=None):
    """Draw face boxes and targeting overlay on frame."""
    frame_center_x, frame_center_y = frame_center

    # Draw crosshair at frame center
    cv2.line(
        frame,
        (frame_center_x - 30, frame_center_y),
        (frame_center_x + 30, frame_center_y),
        (0, 255, 0),
        2,
    )
    cv2.line(
        frame,
        (frame_center_x, frame_center_y - 30),
        (frame_center_x, frame_center_y + 30),
        (0, 255, 0),
        2,
    )

    # Draw deadzone circle
    cv2.circle(frame, (frame_center_x, frame_center_y), DEADZONE_PIXELS, (0, 255, 0), 1)

    for i, (top, right, bottom, left) in enumerate(face_locations):
        color = (0, 0, 255) if i == 0 else (128, 128, 128)
        cv2.rectangle(frame, (left, top), (right, bottom), color, 2)

        face_x, face_y = get_face_center((top, right, bottom, left))
        cv2.circle(frame, (face_x, face_y), 5, color, -1)

        if i == 0:
            cv2.line(
                frame,
                (frame_center_x, frame_center_y),
                (face_x, face_y),
                (255, 0, 0),
                2,
            )

    # Status text
    if face_locations:
        cv2.putText(
            frame,
            f"Tracking face",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2,
        )
    else:
        cv2.putText(
            frame,
            "No face - holding position",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 165, 255),
            2,
        )

    if status_text:
        cv2.putText(
            frame,
            status_text,
            (10, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            1,
        )

    return frame


# =============================================================================
# Main program
# =============================================================================

print("=" * 70)
print("Face Tracking Robot (Direct Control)")
print("=" * 70)
print(f"\nConfiguration:")
print(f"  Horizontal: Motor {HORIZONTAL_MOTOR} (sign={HORIZONTAL_SIGN})")
print(f"  Vertical:   Motor {VERTICAL_MOTOR} (sign={VERTICAL_SIGN})")
print(f"  Max speed:  {MAX_SPEED_DEG_PER_SEC}°/sec")
print(f"  Gain:       {GAIN}")

# Load Haar cascade for face detection
print("\nLoading face detection model...")
face_cascade = cv2.CascadeClassifier(
    cv2.data.haarcascades + "haarcascade_frontalface_default.xml"
)
if face_cascade.empty():
    print("✗ Error: Could not load face detection model")
    sys.exit(1)
print("✓ Face detection model loaded")

# Connect to motor bus
arm = make_bus(FOLLOWER_PORT, "arm")
arm_connected = connect_bus(arm, "arm", handshake=True)

if not arm_connected:
    print("\n✗ Error: Could not connect to arm. Exiting.")
    sys.exit(1)

# Configure motors
print("\nConfiguring motors...")
try:
    arm.enable_torque()
    print("✓ Torque enabled")
except Exception as e:
    print(f"✗ Error: Could not enable torque: {e}")
    sys.exit(1)

# Load calibration
print("Loading calibration...")
try:
    arm.calibration = arm.read_calibration()
    print("✓ Calibration loaded")
except Exception as e:
    print(f"⚠ Warning: Could not load calibration: {e}")

# Read initial joint positions
current_positions = {}
for i in MOTOR_IDS:
    current_positions[i] = arm.read("Present_Position", f"arm_{i}", normalize=True)

print(f"Initial positions: {[f'{current_positions[i]:.1f}°' for i in MOTOR_IDS]}")

# Initialize camera
print("\nInitializing camera...")
cap_test = cv2.VideoCapture(CAMERA_INDEX)
camera_connected = cap_test.isOpened()
if camera_connected:
    actual_width = int(cap_test.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_height = int(cap_test.get(cv2.CAP_PROP_FRAME_HEIGHT))
    cap_test.release()
    print(f"✓ Camera detected ({actual_width}x{actual_height})")

    camera_thread = threading.Thread(
        target=camera_capture_loop, args=(CAMERA_INDEX, stop_camera), daemon=True
    )
    camera_thread.start()
    print("✓ Camera capture thread started")
else:
    cap_test.release()
    print("✗ Error: Could not open camera. Exiting.")
    sys.exit(1)

# Frame center
frame_center = (CAMERA_WIDTH // 2, CAMERA_HEIGHT // 2)

# Target positions for controlled joints
target_h = current_positions[HORIZONTAL_MOTOR]
target_v = current_positions[VERTICAL_MOTOR]

print("\n" + "=" * 70)
print("Face Tracking Active")
print("=" * 70)
print("Press 'q' or Ctrl+C to stop")
print("\nIf tracking direction is wrong, edit HORIZONTAL_SIGN or VERTICAL_SIGN")
print()

last_detection_time = 0
last_face_locations = []
DETECTION_INTERVAL = 0.1
last_loop_time = time.time()

# Smoothed face position (for reducing jitter)
smoothed_face_x = CAMERA_WIDTH // 2
smoothed_face_y = CAMERA_HEIGHT // 2

# Smoothed motor commands
smoothed_target_h = target_h
smoothed_target_v = target_v

try:
    while not stop_camera.is_set():
        loop_start = time.time()
        dt = loop_start - last_loop_time
        last_loop_time = loop_start

        # Calculate max movement this frame based on speed limit
        max_move = MAX_SPEED_DEG_PER_SEC * dt

        # Get latest frame
        with frame_lock:
            frame = latest_frame.copy() if latest_frame is not None else None

        if frame is None:
            time.sleep(0.01)
            continue

        # Run face detection periodically
        if (loop_start - last_detection_time) >= DETECTION_INTERVAL:
            last_face_locations = detect_faces(frame, face_cascade)
            last_detection_time = loop_start

        face_locations = last_face_locations
        status_text = None

        if face_locations:
            raw_face_x, raw_face_y = get_face_center(face_locations[0])

            # Smooth the face position (exponential moving average)
            smoothed_face_x = (
                FACE_SMOOTHING * smoothed_face_x + (1 - FACE_SMOOTHING) * raw_face_x
            )
            smoothed_face_y = (
                FACE_SMOOTHING * smoothed_face_y + (1 - FACE_SMOOTHING) * raw_face_y
            )

            frame_center_x, frame_center_y = frame_center

            # Calculate offset from center using smoothed position
            offset_x = (
                smoothed_face_x - frame_center_x
            )  # Positive = face is to the right
            offset_y = (
                smoothed_face_y - frame_center_y
            )  # Positive = face is below center

            # Apply deadzone
            if abs(offset_x) < DEADZONE_PIXELS:
                offset_x = 0
            if abs(offset_y) < DEADZONE_PIXELS:
                offset_y = 0

            # Calculate desired movement
            move_h = offset_x * GAIN * HORIZONTAL_SIGN
            move_v = offset_y * GAIN * VERTICAL_SIGN

            # Clamp to max speed
            move_h = np.clip(move_h, -max_move, max_move)
            move_v = np.clip(move_v, -max_move, max_move)

            # Update targets
            target_h += move_h
            target_v += move_v

            # Clamp to joint limits to prevent overload errors
            h_min, h_max = JOINT_LIMITS[HORIZONTAL_MOTOR]
            v_min, v_max = JOINT_LIMITS[VERTICAL_MOTOR]
            target_h = np.clip(target_h, h_min, h_max)
            target_v = np.clip(target_v, v_min, v_max)

            # Smooth the output commands (reduces jitter in motor movement)
            smoothed_target_h = (
                OUTPUT_SMOOTHING * smoothed_target_h + (1 - OUTPUT_SMOOTHING) * target_h
            )
            smoothed_target_v = (
                OUTPUT_SMOOTHING * smoothed_target_v + (1 - OUTPUT_SMOOTHING) * target_v
            )

            status_text = f"Offset: ({offset_x:+.0f}, {offset_y:+.0f})  Target: ({smoothed_target_h:.1f}°, {smoothed_target_v:.1f}°)"

            # Write smoothed values to motors
            arm.write(
                "Goal_Position",
                f"arm_{HORIZONTAL_MOTOR}",
                smoothed_target_h,
                normalize=True,
            )
            arm.write(
                "Goal_Position",
                f"arm_{VERTICAL_MOTOR}",
                smoothed_target_v,
                normalize=True,
            )

        # Draw overlay and display
        display_frame = draw_overlay(frame, face_locations, frame_center, status_text)
        cv2.imshow("Face Tracking", display_frame)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            print("\n'q' pressed, stopping...")
            break

        # Maintain loop rate (~50Hz)
        elapsed = time.time() - loop_start
        if elapsed < 0.02:
            time.sleep(0.02 - elapsed)

except KeyboardInterrupt:
    print("\n\nStopping...")
except Exception as e:
    print(f"\n✗ Error during operation: {e}")
    import traceback

    traceback.print_exc()
finally:
    print("Cleaning up...")
    stop_camera.set()
    if camera_thread is not None and camera_thread.is_alive():
        camera_thread.join(timeout=1.0)
    cv2.destroyAllWindows()
    try:
        arm.disable_torque()
        print("✓ Torque disabled")
    except Exception:
        pass
    try:
        arm.disconnect()
        print("✓ Disconnected")
    except Exception:
        pass
    print("✓ Done")
