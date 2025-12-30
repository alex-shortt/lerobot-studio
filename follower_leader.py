from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus
import time
import sys

# adjust ports and motor ids to match your setup
LEADER_PORT = "/dev/tty.usbmodem5AE60530061"
FOLLOWER_PORT = "/dev/tty.usbmodem5AE60805191"

# assuming 6 joints, ids 1-6 on both arms
MOTOR_IDS = [1, 2, 3, 4, 5, 6]
MOTOR_MODEL = "sts3215"


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
            print(f"  Continuing without {name}...")
            return False
        else:
            raise


leader = make_bus(LEADER_PORT, "leader")
follower = make_bus(FOLLOWER_PORT, "follower")

# Try to connect both buses
leader_connected = connect_bus(leader, "leader", handshake=True)
follower_connected = connect_bus(follower, "follower", handshake=True)

if not follower_connected:
    print(
        "\n✗ Error: Follower bus connection failed. Cannot continue without follower."
    )
    sys.exit(1)

if not leader_connected:
    print(
        "\n⚠ Warning: Leader bus not connected. Follower will not track leader movements."
    )
    print(
        "  Make sure the leader arm is connected and powered, then restart the script."
    )
    sys.exit(1)

# leader stays loose (torque off), follower tracks
print("\nConfiguring motors...")
try:
    leader.disable_torque()
    print("✓ Leader torque disabled")
except Exception as e:
    print(f"⚠ Warning: Could not disable leader torque: {e}")

try:
    follower.enable_torque()
    print("✓ Follower torque enabled")
except Exception as e:
    print(f"✗ Error: Could not enable follower torque: {e}")
    sys.exit(1)

# load calibrations
print("\nLoading calibrations...")
try:
    leader.calibration = leader.read_calibration()
    print("✓ Leader calibration loaded")
except Exception as e:
    print(f"⚠ Warning: Could not load leader calibration: {e}")

try:
    follower.calibration = follower.read_calibration()
    print("✓ Follower calibration loaded")
except Exception as e:
    print(f"✗ Error: Could not load follower calibration: {e}")
    sys.exit(1)

# Initialization phase: gradually move follower to leader position
print("\n" + "=" * 70)
print("Initialization Phase")
print("=" * 70)
print("Gradually moving follower to match leader position...")

# Configuration for initialization
INIT_MAX_SPEED = 20.0  # Maximum speed in degrees per second (reduced for safety)
INIT_UPDATE_RATE = 50  # Hz
INIT_POSITION_TOLERANCE = 3.0  # Degrees - close enough to start tracking
INIT_TIMEOUT = 15.0  # Maximum time for initialization (seconds)

# Read initial positions for display
follower_start_positions = {}
for i in MOTOR_IDS:
    follower_start_positions[i] = follower.read(
        "Present_Position", f"follower_{i}", normalize=True
    )

print(
    f"\nInitial follower positions: {[f'{follower_start_positions[i]:.1f}°' for i in MOTOR_IDS]}"
)

# Gradually move follower to leader position
# Track current commanded positions to ensure smooth movement
follower_current_targets = {i: follower_start_positions[i] for i in MOTOR_IDS}

init_start = time.time()
dt = 1.0 / INIT_UPDATE_RATE
all_close = False
max_error = 0.0  # Initialize for timeout case
last_print_time = 0.0

while not all_close and (time.time() - init_start) < INIT_TIMEOUT:
    loop_start = time.time()
    elapsed = time.time() - init_start

    all_close = True
    max_error = 0.0

    for i in MOTOR_IDS:
        # Read current actual position of follower
        follower_actual_pos = follower.read("Present_Position", f"follower_{i}", normalize=True)
        
        # Read current leader position (it may be moving, so read it fresh each time)
        leader_target_pos = leader.read("Present_Position", f"leader_{i}", normalize=True)
        
        # Get our current target position (what we're commanding)
        current_target = follower_current_targets[i]
        
        # Calculate error from actual position to leader position
        error = abs(leader_target_pos - follower_actual_pos)
        max_error = max(max_error, error)

        if error > INIT_POSITION_TOLERANCE:
            all_close = False
            
            # Calculate how much we can move this step (speed limit)
            max_step = INIT_MAX_SPEED * dt
            
            # Calculate distance from current target to leader target
            distance_to_leader = leader_target_pos - current_target
            
            # Move our target toward the leader, but cap the step size
            if abs(distance_to_leader) > max_step:
                # Move by max_step toward leader
                direction = 1 if distance_to_leader > 0 else -1
                new_target = current_target + direction * max_step
            else:
                # Close enough, move target directly to leader
                new_target = leader_target_pos
            
            # Update our target and command the motor
            follower_current_targets[i] = new_target
            follower.write("Goal_Position", f"follower_{i}", new_target, normalize=True)
        else:
            # Already close enough, keep target at leader position
            follower_current_targets[i] = leader_target_pos
            follower.write("Goal_Position", f"follower_{i}", leader_target_pos, normalize=True)

    # Print progress every 0.5 seconds
    if elapsed - last_print_time >= 0.5:
        print(
            f"  Initializing... Max error: {max_error:.1f}° (target: <{INIT_POSITION_TOLERANCE}°)"
        )
        last_print_time = elapsed

    # Sleep to maintain update rate
    sleep_time = dt - (time.time() - loop_start)
    if sleep_time > 0:
        time.sleep(sleep_time)

if all_close:
    print(
        f"✓ Initialization complete! All motors within {INIT_POSITION_TOLERANCE}° of target"
    )
else:
    print(
        f"⚠ Initialization timeout. Starting tracking anyway (max error: {max_error:.1f}°)"
    )

print("\n" + "=" * 70)
print("Follower-Leader Control Active")
print("=" * 70)
print("Press Ctrl+C to stop")
print()

try:
    while True:
        # read all leader positions and write to corresponding follower motors
        for i in MOTOR_IDS:
            leader_pos = leader.read("Present_Position", f"leader_{i}", normalize=True)
            follower.write("Goal_Position", f"follower_{i}", leader_pos, normalize=True)

        time.sleep(0.02)  # 50hz
except KeyboardInterrupt:
    print("\n\nStopping...")
except Exception as e:
    print(f"\n✗ Error during operation: {e}")
finally:
    print("Disabling torque and disconnecting...")
    try:
        follower.disable_torque()
    except Exception:
        pass
    try:
        leader.disconnect()
    except Exception:
        pass
    try:
        follower.disconnect()
    except Exception:
        pass
    print("✓ Done")
