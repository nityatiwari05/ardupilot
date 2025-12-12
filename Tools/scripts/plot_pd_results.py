#!/usr/bin/env python3
"""
Plot and analyze PD controller test results from ArduPilot logs
"""

import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from pymavlink import mavutil

class PDLogAnalyzer:
    def __init__(self, logfile):
        """Initialize log file"""
        self.logfile = logfile
        print(f"Loading log: {logfile}")
        self.mlog = mavutil.mavlink_connection(logfile)
        self.data = {}
        
    def load_ang_data(self):
        """Load ANG (attitude) log data"""
        print("Loading ANG messages...")
        ang_data = {
            'time': [],
            'control_roll': [],
            'roll': [],
            'control_pitch': [],
            'pitch': [],
            'control_yaw': [],
            'yaw': [],
            'sensor_dt': [],
            'pitch_outer_D': [],
            'pitch_deriv_error': []
        }
        
        while True:
            msg = self.mlog.recv_match(type='ANG', blocking=False)
            if msg is None:
                break
            
            ang_data['time'].append(msg.TimeUS * 1e-6)  # Convert to seconds
            ang_data['control_roll'].append(msg.DesRoll)
            ang_data['roll'].append(msg.Roll)
            ang_data['control_pitch'].append(msg.DesPitch)
            ang_data['pitch'].append(msg.Pitch)
            ang_data['control_yaw'].append(msg.DesYaw)
            ang_data['yaw'].append(msg.Yaw)
            ang_data['sensor_dt'].append(msg.Dt)
            ang_data['pitch_outer_D'].append(msg.PitchD)
            ang_data['pitch_deriv_error'].append(msg.PitchEd)
        
        # Convert to numpy arrays
        for key in ang_data:
            ang_data[key] = np.array(ang_data[key])
        
        # Make time relative to start
        if len(ang_data['time']) > 0:
            ang_data['time'] = ang_data['time'] - ang_data['time'][0]
        
        self.data['ANG'] = ang_data
        print(f"  Loaded {len(ang_data['time'])} ANG messages")
        return ang_data
    
    def load_rate_data(self):
        """Load RATE log data"""
        print("Loading RATE messages...")
        rate_data = {
            'time': [],
            'control_roll': [],
            'roll': [],
            'roll_out': [],
            'control_pitch': [],
            'pitch': [],
            'pitch_out': [],
            'control_yaw': [],
            'yaw': [],
            'yaw_out': []
        }
        
        while True:
            msg = self.mlog.recv_match(type='RATE', blocking=False)
            if msg is None:
                break
            
            rate_data['time'].append(msg.TimeUS * 1e-6)
            rate_data['control_roll'].append(msg.RDes)
            rate_data['roll'].append(msg.R)
            rate_data['roll_out'].append(msg.ROut)
            rate_data['control_pitch'].append(msg.PDes)
            rate_data['pitch'].append(msg.P)
            rate_data['pitch_out'].append(msg.POut)
            rate_data['control_yaw'].append(msg.YDes)
            rate_data['yaw'].append(msg.Y)
            rate_data['yaw_out'].append(msg.YOut)
        
        # Convert to numpy arrays
        for key in rate_data:
            rate_data[key] = np.array(rate_data[key])
        
        # Make time relative to start
        if len(rate_data['time']) > 0:
            rate_data['time'] = rate_data['time'] - rate_data['time'][0]
        
        self.data['RATE'] = rate_data
        print(f"  Loaded {len(rate_data['time'])} RATE messages")
        return rate_data
    
    def plot_step_response(self, title="Step Response", save_path=None):
        """Plot step response analysis"""
        if 'ANG' not in self.data:
            self.load_ang_data()
        
        ang = self.data['ANG']
        
        fig, axes = plt.subplots(3, 1, figsize=(12, 10))
        fig.suptitle(title, fontsize=14, fontweight='bold')
        
        # Plot 1: Pitch angle (desired vs actual)
        ax1 = axes[0]
        ax1.plot(ang['time'], ang['control_pitch'], 'b--', label='Desired Pitch', linewidth=2)
        ax1.plot(ang['time'], ang['pitch'], 'r-', label='Actual Pitch', linewidth=1.5)
        ax1.set_ylabel('Pitch Angle (deg)', fontsize=11)
        ax1.set_title('Pitch Angle Response', fontsize=12)
        ax1.grid(True, alpha=0.3)
        ax1.legend(loc='best')
        
        # Plot 2: Pitch rate (from RATE messages if available)
        ax2 = axes[1]
        if 'RATE' in self.data:
            rate = self.data['RATE']
            ax2.plot(rate['time'], rate['control_pitch'], 'b--', label='Desired Rate', linewidth=2)
            ax2.plot(rate['time'], rate['pitch'], 'r-', label='Actual Rate', linewidth=1.5)
            ax2.set_ylabel('Pitch Rate (deg/s)', fontsize=11)
            ax2.set_title('Pitch Rate Response', fontsize=12)
        else:
            ax2.text(0.5, 0.5, 'RATE data not available', 
                    ha='center', va='center', transform=ax2.transAxes)
        ax2.grid(True, alpha=0.3)
        ax2.legend(loc='best')
        
        # Plot 3: PD terms
        ax3 = axes[2]
        ax3_twin = ax3.twinx()
        
        # D term contribution
        ax3.plot(ang['time'], ang['pitch_outer_D'], 'g-', label='D Term (Kd*ed)', linewidth=1.5)
        ax3.set_ylabel('D Term Contribution (deg/s)', fontsize=11, color='g')
        ax3.tick_params(axis='y', labelcolor='g')
        
        # Derivative error
        ax3_twin.plot(ang['time'], ang['pitch_deriv_error'], 'm-', label='Derivative Error (ed)', linewidth=1.5, alpha=0.7)
        ax3_twin.set_ylabel('Derivative Error (rad/s)', fontsize=11, color='m')
        ax3_twin.tick_params(axis='y', labelcolor='m')
        
        ax3.set_xlabel('Time (s)', fontsize=11)
        ax3.set_title('PD Controller Terms', fontsize=12)
        ax3.grid(True, alpha=0.3)
        
        # Combine legends
        lines1, labels1 = ax3.get_legend_handles_labels()
        lines2, labels2 = ax3_twin.get_legend_handles_labels()
        ax3.legend(lines1 + lines2, labels1 + labels2, loc='best')
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=150, bbox_inches='tight')
            print(f"Saved plot to {save_path}")
        else:
            plt.show()
    
    def plot_sinusoid_response(self, title="Sinusoid Tracking", save_path=None):
        """Plot sinusoid tracking analysis"""
        if 'ANG' not in self.data:
            self.load_ang_data()
        
        ang = self.data['ANG']
        
        fig, axes = plt.subplots(3, 1, figsize=(12, 10))
        fig.suptitle(title, fontsize=14, fontweight='bold')
        
        # Plot 1: Pitch tracking
        ax1 = axes[0]
        ax1.plot(ang['time'], ang['control_pitch'], 'b--', label='Desired Pitch', linewidth=2)
        ax1.plot(ang['time'], ang['pitch'], 'r-', label='Actual Pitch', linewidth=1.5)
        ax1.set_ylabel('Pitch Angle (deg)', fontsize=11)
        ax1.set_title('Pitch Tracking', fontsize=12)
        ax1.grid(True, alpha=0.3)
        ax1.legend(loc='best')
        
        # Plot 2: Tracking error
        ax2 = axes[1]
        error = ang['pitch'] - ang['control_pitch']
        ax2.plot(ang['time'], error, 'r-', label='Tracking Error', linewidth=1.5)
        ax2.axhline(y=0, color='k', linestyle='--', alpha=0.3)
        ax2.set_ylabel('Error (deg)', fontsize=11)
        ax2.set_title('Tracking Error', fontsize=12)
        ax2.grid(True, alpha=0.3)
        ax2.legend(loc='best')
        
        # Plot 3: Frequency domain (FFT)
        ax3 = axes[2]
        if len(error) > 100:
            # Compute FFT
            dt = np.mean(np.diff(ang['time']))
            fft_vals = np.fft.fft(error)
            fft_freq = np.fft.fftfreq(len(error), dt)
            
            # Plot magnitude spectrum
            ax3.plot(fft_freq[:len(fft_freq)//2], np.abs(fft_vals[:len(fft_vals)//2]), 'b-', linewidth=1.5)
            ax3.set_xlabel('Frequency (Hz)', fontsize=11)
            ax3.set_ylabel('Magnitude', fontsize=11)
            ax3.set_title('Error Frequency Spectrum', fontsize=12)
            ax3.grid(True, alpha=0.3)
            ax3.set_xlim(0, 5)  # Focus on 0-5 Hz
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=150, bbox_inches='tight')
            print(f"Saved plot to {save_path}")
        else:
            plt.show()
    
    def compute_metrics(self):
        """Compute performance metrics"""
        if 'ANG' not in self.data:
            self.load_ang_data()
        
        ang = self.data['ANG']
        
        # Compute error
        error = ang['pitch'] - ang['control_pitch']
        
        metrics = {
            'rmse': np.sqrt(np.mean(error**2)),
            'max_error': np.max(np.abs(error)),
            'settling_time': None,  # Would need to detect step
            'overshoot': None,  # Would need to detect step
            'mean_d_term': np.mean(np.abs(ang['pitch_outer_D'])),
            'max_d_term': np.max(np.abs(ang['pitch_outer_D'])),
            'mean_deriv_error': np.mean(np.abs(ang['pitch_deriv_error'])),
            'max_deriv_error': np.max(np.abs(ang['pitch_deriv_error']))
        }
        
        return metrics
    
    def print_metrics(self):
        """Print computed metrics"""
        metrics = self.compute_metrics()
        print("\n" + "="*60)
        print("Performance Metrics")
        print("="*60)
        print(f"RMSE:              {metrics['rmse']:.4f} deg")
        print(f"Max Error:         {metrics['max_error']:.4f} deg")
        print(f"Mean |D Term|:     {metrics['mean_d_term']:.4f} deg/s")
        print(f"Max |D Term|:      {metrics['max_d_term']:.4f} deg/s")
        print(f"Mean |Deriv Error|: {metrics['mean_deriv_error']:.4f} rad/s")
        print(f"Max |Deriv Error|: {metrics['max_deriv_error']:.4f} rad/s")
        print("="*60)


def compare_logs(logfiles, labels=None, test_type='step'):
    """Compare multiple log files"""
    if labels is None:
        labels = [f"Test {i+1}" for i in range(len(logfiles))]
    
    fig, axes = plt.subplots(2, 1, figsize=(12, 8))
    
    for logfile, label in zip(logfiles, labels):
        analyzer = PDLogAnalyzer(logfile)
        analyzer.load_ang_data()
        ang = analyzer.data['ANG']
        
        # Plot pitch response
        axes[0].plot(ang['time'], ang['control_pitch'], '--', alpha=0.5, label=f'{label} (desired)')
        axes[0].plot(ang['time'], ang['pitch'], '-', linewidth=1.5, label=f'{label} (actual)')
        
        # Plot D term
        axes[1].plot(ang['time'], ang['pitch_outer_D'], '-', linewidth=1.5, label=f'{label} D-term')
    
    axes[0].set_ylabel('Pitch Angle (deg)', fontsize=11)
    axes[0].set_title('Pitch Response Comparison', fontsize=12)
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(loc='best')
    
    axes[1].set_xlabel('Time (s)', fontsize=11)
    axes[1].set_ylabel('D Term Contribution (deg/s)', fontsize=11)
    axes[1].set_title('D Term Comparison', fontsize=12)
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(loc='best')
    
    plt.tight_layout()
    plt.show()


def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='Plot PD Controller Test Results')
    parser.add_argument('logfile', help='Log file path (.BIN or .tlog)')
    parser.add_argument('--type', choices=['step', 'sinusoid', 'auto'],
                       default='auto', help='Test type')
    parser.add_argument('--save', help='Save plot to file')
    parser.add_argument('--metrics', action='store_true',
                       help='Print performance metrics')
    
    args = parser.parse_args()
    
    # Expand user path and resolve relative paths
    logfile = os.path.expanduser(args.logfile)
    if not os.path.isabs(logfile):
        # Try relative to current directory, then relative to ardupilot root
        if not os.path.exists(logfile):
            ardupilot_root = os.path.join(os.path.dirname(__file__), '..', '..')
            ardupilot_root = os.path.abspath(ardupilot_root)
            alt_path = os.path.join(ardupilot_root, 'logs', os.path.basename(logfile))
            if os.path.exists(alt_path):
                logfile = alt_path
    
    if not os.path.exists(logfile):
        print(f"Error: Log file not found: {args.logfile}")
        print(f"Tried: {logfile}")
        print("\nHint: Logs are typically in ~/ardupilot/logs/")
        print("Example: python3 plot_pd_results.py ~/ardupilot/logs/00000003.BIN --type step")
        sys.exit(1)
    
    args.logfile = logfile
    
    analyzer = PDLogAnalyzer(args.logfile)
    analyzer.load_ang_data()
    analyzer.load_rate_data()
    
    if args.metrics:
        analyzer.print_metrics()
    
    # Auto-detect test type or use specified
    if args.type == 'auto':
        # Simple heuristic: check if pitch command is sinusoidal
        ang = analyzer.data['ANG']
        if len(ang['control_pitch']) > 50:
            # Check for sinusoidal pattern
            pitch_diff = np.diff(ang['control_pitch'])
            zero_crossings = np.sum(np.diff(np.sign(pitch_diff)) != 0)
            if zero_crossings > 5:
                args.type = 'sinusoid'
            else:
                args.type = 'step'
    
    if args.type == 'step':
        analyzer.plot_step_response(save_path=args.save)
    elif args.type == 'sinusoid':
        analyzer.plot_sinusoid_response(save_path=args.save)
    
    print("\nAnalysis complete!")


if __name__ == '__main__':
    main()

