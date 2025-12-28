#!/usr/bin/env python3
"""
Simple script to control a motor with a sine wave pattern.

This script demonstrates basic motor control by moving a motor in a sinusoidal
pattern over time. Useful for testing motor functionality and understanding
position control.

Usage:
    python sine_wave_motor_control.py

Configuration:
    Edit the MOTOR_CONFIG section below to match your hardware setup.
"""

import math
import sys
import time
from pathlib import Path

# Add src directory to path so we can import lerobot
sys.path.insert(0, str(Path(__file__).parent / "src"))

from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus

# ============================================================================
# MOTOR CONFIGURATION - Edit these values to match your setup
# ============================================================================

MOTOR_CONFIG = {
    # Motor bus configuration
    "port": "/dev/tty.usbmodem5AE60805191",  # USB port for motor connection
    "motor_id": 6,  # Motor ID (set on the physical motor)
    "motor_model": "sts3215",  # Motor model (sts3215, sts3250, sm8512bl, scs0009)
    "motor_name": "test_motor",  # Friendly name for the motor

    # Sine wave parameters
    "frequency": 0.5,  # Frequency in Hz (0.5 = one complete cycle every 2 seconds)
    "amplitude": 30.0,  # Amplitude in degrees (±30° from center)
    "center_position": 0.0,  # Center position in degrees
    "duration": 10.0,  # Total duration to run in seconds (0 = infinite)

    # Control parameters
    "update_rate": 50,  # Control loop frequency in Hz
}

# ============================================================================
# MAIN SCRIPT
# ============================================================================


def main():
    print("=" * 70)
    print("LeRobot Sine Wave Motor Control")
    print("=" * 70)
    print(f"\nMotor Configuration:")
    print(f"  Port: {MOTOR_CONFIG['port']}")
    print(f"  Motor: {MOTOR_CONFIG['motor_name']} (ID: {MOTOR_CONFIG['motor_id']})")
    print(f"  Model: {MOTOR_CONFIG['motor_model']}")
    print(f"\nSine Wave Parameters:")
    print(f"  Frequency: {MOTOR_CONFIG['frequency']} Hz")
    print(f"  Amplitude: ±{MOTOR_CONFIG['amplitude']}°")
    print(f"  Center: {MOTOR_CONFIG['center_position']}°")
    print(f"  Duration: {MOTOR_CONFIG['duration']}s" + (" (infinite)" if MOTOR_CONFIG['duration'] == 0 else ""))
    print(f"  Update Rate: {MOTOR_CONFIG['update_rate']} Hz")
    print()

    # Initialize motor bus
    print("Initializing motor bus...")
    motor_bus = FeetechMotorsBus(
        port=MOTOR_CONFIG["port"],
        motors={
            MOTOR_CONFIG["motor_name"]: Motor(
                id=MOTOR_CONFIG["motor_id"],
                model=MOTOR_CONFIG["motor_model"],
                norm_mode=MotorNormMode.DEGREES,
            )
        },
    )

    try:
        # Connect to motors
        print("Connecting to motor...")
        motor_bus.connect(handshake=True)
        print("✓ Motor connected successfully!")

        # Read calibration from motor (required for normalize=True)
        print("Reading motor calibration...")
        calibration = motor_bus.read_calibration()
        motor_bus.calibration = calibration
        print("✓ Calibration loaded!")

        # Enable torque
        print("Enabling torque...")
        motor_bus.enable_torque()
        print("✓ Torque enabled!")

        # Read initial position
        initial_pos = motor_bus.read(
            "Present_Position",
            MOTOR_CONFIG["motor_name"],
            normalize=True
        )
        print(f"✓ Initial position: {initial_pos:.2f}°")
        print()

        # Run sine wave control loop
        print("Starting sine wave motion... (Press Ctrl+C to stop)")
        print("-" * 70)

        start_time = time.time()
        dt = 1.0 / MOTOR_CONFIG["update_rate"]
        duration = MOTOR_CONFIG["duration"]

        iteration = 0
        while True:
            loop_start = time.time()
            elapsed = loop_start - start_time

            # Check if duration has been reached
            if duration > 0 and elapsed >= duration:
                print("\n✓ Duration reached. Stopping...")
                break

            # Calculate target position using sine wave
            # position(t) = center + amplitude * sin(2π * frequency * t)
            target_pos = (
                MOTOR_CONFIG["center_position"]
                + MOTOR_CONFIG["amplitude"] * math.sin(2 * math.pi * MOTOR_CONFIG["frequency"] * elapsed)
            )

            # Send position command to motor
            motor_bus.write(
                "Goal_Position",
                MOTOR_CONFIG["motor_name"],
                target_pos,
                normalize=True
            )

            # Read current position for feedback
            if iteration % 10 == 0:  # Print every 10 iterations to reduce spam
                current_pos = motor_bus.read(
                    "Present_Position",
                    MOTOR_CONFIG["motor_name"],
                    normalize=True
                )
                print(
                    f"t={elapsed:6.2f}s | Target: {target_pos:7.2f}° | "
                    f"Actual: {current_pos:7.2f}° | Error: {abs(target_pos - current_pos):5.2f}°"
                )

            iteration += 1

            # Sleep to maintain update rate
            sleep_time = dt - (time.time() - loop_start)
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\n\n✓ Interrupted by user. Stopping...")

    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()

    finally:
        # Return to center position
        print("\nReturning to center position...")
        try:
            motor_bus.write(
                "Goal_Position",
                MOTOR_CONFIG["motor_name"],
                MOTOR_CONFIG["center_position"],
                normalize=True
            )
            time.sleep(1.0)  # Wait for motor to reach center
        except Exception:
            pass

        # Disable torque and disconnect
        print("Disabling torque...")
        try:
            motor_bus.disable_torque()
            print("✓ Torque disabled!")
        except Exception:
            pass

        print("Disconnecting...")
        try:
            motor_bus.disconnect()
            print("✓ Disconnected!")
        except Exception:
            pass

        print("\n" + "=" * 70)
        print("Motor control session ended.")
        print("=" * 70)


if __name__ == "__main__":
    main()
