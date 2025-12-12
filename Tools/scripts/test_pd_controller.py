#!/usr/bin/env python3
"""
SITL Test Script for Pitch Angle PD Controller
Tests the PD controller implementation with various KD values and test profiles
"""

import time
import sys
import os
from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mavlink2

class PDControllerTester:
    def __init__(self, connection_string='udp:127.0.0.1:14550'):
        """Initialize MAVLink connection"""
        print(f"Connecting to {connection_string}...")
        self.master = mavutil.mavlink_connection(connection_string)
        self.master.wait_heartbeat()
        print("Connected!")
        
    def set_param(self, param_name, value, max_retries=5):
        """Set a parameter value with retry logic"""
        print(f"Setting {param_name} = {value}")
        for attempt in range(max_retries):
            # Clear any pending messages
            self.master.recv_match(blocking=False)
            
            # Send parameter set
            self.master.param_set_send(param_name, value)
            time.sleep(0.3)
            
            # Fetch and verify
            self.master.param_fetch_one(param_name)
            msg = None
            for _ in range(10):  # Try multiple times to get response
                msg = self.master.recv_match(type='PARAM_VALUE', blocking=False)
                if msg and msg.param_id == param_name:
                    break
                time.sleep(0.1)
            
            if msg and abs(msg.param_value - value) < 0.001:
                print(f"  ✓ {param_name} = {msg.param_value}")
                return True
            elif attempt < max_retries - 1:
                print(f"  Retrying... (attempt {attempt + 1}/{max_retries})")
                time.sleep(0.5)
        
        print(f"  ✗ Failed to set {param_name} after {max_retries} attempts")
        return False
    
    def save_params(self):
        """Save parameters"""
        print("Saving parameters...")
        # Parameters are auto-saved in SITL, but we can verify they're set
        time.sleep(0.5)
    
    def arm(self):
        """Arm the vehicle"""
        print("Arming vehicle...")
        self.master.arducopter_arm()
        time.sleep(2)
        return True
    
    def disarm(self):
        """Disarm the vehicle"""
        print("Disarming vehicle...")
        self.master.arducopter_disarm()
        time.sleep(1)
    
    def set_mode(self, mode_name):
        """Set flight mode"""
        print(f"Setting mode to {mode_name}...")
        mode_id = self.master.mode_mapping()[mode_name]
        self.master.set_mode(mode_id)
        time.sleep(1)
    
    def takeoff(self, altitude=5):
        """Takeoff to altitude"""
        print(f"Taking off to {altitude}m...")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavlink2.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, altitude)
        time.sleep(5)
    
    def set_attitude_target(self, roll=0, pitch=0, yaw=0, yaw_rate=0):
        """Set attitude target using SET_ATTITUDE_TARGET"""
        # Convert to radians
        import math
        roll_rad = math.radians(roll)
        pitch_rad = math.radians(pitch)
        yaw_rad = math.radians(yaw)
        
        # Create quaternion from Euler angles (ZYX order)
        cr = math.cos(roll_rad * 0.5)
        sr = math.sin(roll_rad * 0.5)
        cp = math.cos(pitch_rad * 0.5)
        sp = math.sin(pitch_rad * 0.5)
        cy = math.cos(yaw_rad * 0.5)
        sy = math.sin(yaw_rad * 0.5)
        
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        
        self.master.mav.set_attitude_target_send(
            0,  # time_boot_ms
            self.master.target_system,
            self.master.target_component,
            0b00000111,  # type_mask: ignore rates, use attitude
            [qw, qx, qy, qz],  # q
            0, 0, 0,  # body roll/pitch/yaw rates (ignored)
            0, 0)  # thrust
    
    def wait_settle(self, duration=10):
        """Wait for system to settle"""
        print(f"Waiting {duration}s for system to settle...")
        time.sleep(duration)
    
    def run_step_test(self, kd_value, test_name, pitch_step=10, settle_time=15):
        """Run a step response test"""
        print(f"\n{'='*60}")
        print(f"Test: {test_name} (KD={kd_value})")
        print(f"{'='*60}")
        
        # Set KD parameter (critical - must succeed)
        kd_set = self.set_param('ATC_ANG_PIT_D', kd_value)
        if not kd_set:
            print(f"  WARNING: Failed to set KD={kd_value}, but continuing test...")
            print(f"  The test will run with whatever KD value is currently set.")
        
        # Set filter cutoff (non-critical)
        self.set_param('ATC_ANG_PIT_DF', 15.0)
        
        # Small delay to ensure parameters are applied
        time.sleep(1)
        
        # Arm and takeoff
        if not self.arm():
            print("  ERROR: Failed to arm vehicle")
            return False
        
        self.set_mode('GUIDED')
        time.sleep(2)
        
        self.takeoff(altitude=5)
        time.sleep(5)
        
        # Command initial attitude (level)
        print("Commanding level attitude...")
        self.set_attitude_target(roll=0, pitch=0, yaw=0)
        time.sleep(3)
        
        # Command step input
        print(f"Commanding {pitch_step}° pitch step...")
        self.set_attitude_target(roll=0, pitch=pitch_step, yaw=0)
        
        # Wait for response
        self.wait_settle(settle_time)
        
        # Return to level
        print("Returning to level...")
        self.set_attitude_target(roll=0, pitch=0, yaw=0)
        time.sleep(3)
        
        # Land
        print("Landing...")
        self.set_mode('LAND')
        time.sleep(5)
        
        self.disarm()
        time.sleep(1)
        
        print(f"✓ {test_name} complete")
        return True
    
    def run_sinusoid_test(self, kd_value, frequency, amplitude=5, duration=30):
        """Run a sinusoidal tracking test"""
        print(f"\n{'='*60}")
        print(f"Sinusoid Test: KD={kd_value}, f={frequency}Hz, A={amplitude}°")
        print(f"{'='*60}")
        
        # Set KD parameter
        if not self.set_param('ATC_ANG_PIT_D', kd_value):
            return False
        
        self.set_param('ATC_ANG_PIT_DF', 15.0)
        self.save_params()
        
        # Arm and takeoff
        if not self.arm():
            return False
        
        self.set_mode('GUIDED')
        time.sleep(2)
        
        self.takeoff(altitude=5)
        time.sleep(5)
        
        # Command initial attitude
        self.set_attitude_target(roll=0, pitch=0, yaw=0)
        time.sleep(2)
        
        # Generate sinusoid
        import math
        start_time = time.time()
        dt = 0.1  # 10 Hz command rate
        samples = int(duration / dt)
        
        print(f"Generating {frequency}Hz sinusoid for {duration}s...")
        for i in range(samples):
            t = i * dt
            pitch_cmd = amplitude * math.sin(2 * math.pi * frequency * t)
            self.set_attitude_target(roll=0, pitch=pitch_cmd, yaw=0)
            time.sleep(dt)
            
            # Progress indicator
            if i % 10 == 0:
                elapsed = time.time() - start_time
                print(f"  Progress: {elapsed:.1f}s / {duration}s")
        
        # Return to level
        print("Returning to level...")
        self.set_attitude_target(roll=0, pitch=0, yaw=0)
        time.sleep(2)
        
        # Land
        print("Landing...")
        self.set_mode('LAND')
        time.sleep(5)
        
        self.disarm()
        print(f"✓ Sinusoid test complete")
        return True


def main():
    """Main test sequence"""
    import argparse
    
    parser = argparse.ArgumentParser(description='PD Controller SITL Test Script')
    parser.add_argument('--connection', default='udp:127.0.0.1:14550',
                       help='MAVLink connection string')
    parser.add_argument('--kd-low', type=float, default=0.05,
                       help='Low KD value (default: 0.05)')
    parser.add_argument('--kd-high', type=float, default=1.0,
                       help='High KD value (default: 1.0)')
    
    args = parser.parse_args()
    
    tester = PDControllerTester(args.connection)
    
    try:
        # Only 3 tests: KD=0, KD low, KD high
        print("\n" + "="*60)
        print("Running 3 PD Controller Tests")
        print("="*60)
        
        # Test 1: Baseline (KD=0)
        tester.run_step_test(0.0, "Baseline (KD=0)", pitch_step=10, settle_time=15)
        time.sleep(3)
        
        # Test 2: Low KD
        tester.run_step_test(args.kd_low, f"Low KD ({args.kd_low})", pitch_step=10, settle_time=15)
        time.sleep(3)
        
        # Test 3: High KD
        tester.run_step_test(args.kd_high, f"High KD ({args.kd_high})", pitch_step=10, settle_time=20)
        time.sleep(3)
        
        print("\n" + "="*60)
        print("All tests complete!")
        print("="*60)
        print("\nLogs are saved in: ~/ardupilot/logs/")
        print("To analyze results:")
        print("  python3 plot_pd_results.py ~/ardupilot/logs/0000000X.BIN --type step --metrics")
        print("\nTo compare all 3 tests:")
        print("  python3 compare_pd_logs.py \\")
        print("    ~/ardupilot/logs/0000000X.BIN \\")
        print("    ~/ardupilot/logs/0000000Y.BIN \\")
        print("    ~/ardupilot/logs/0000000Z.BIN \\")
        print("    --labels 'KD=0' 'KD=Low' 'KD=High' --save comparison.png --table")
        
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
        tester.disarm()
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
        tester.disarm()


if __name__ == '__main__':
    main()

