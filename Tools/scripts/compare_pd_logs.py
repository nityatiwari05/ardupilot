#!/usr/bin/env python3
"""
Compare multiple PD controller test logs side-by-side
"""

import sys
import os
import argparse
import numpy as np
import matplotlib.pyplot as plt
from plot_pd_results import PDLogAnalyzer


def compare_step_responses(logfiles, labels=None, save_path=None):
    """Compare step response tests from multiple logs"""
    if labels is None:
        labels = [f"Test {i+1}" for i in range(len(logfiles))]
    
    fig, axes = plt.subplots(3, 1, figsize=(14, 10))
    fig.suptitle('PD Controller Step Response Comparison', fontsize=14, fontweight='bold')
    
    colors = plt.cm.tab10(np.linspace(0, 1, len(logfiles)))
    
    for idx, (logfile, label, color) in enumerate(zip(logfiles, labels, colors)):
        try:
            analyzer = PDLogAnalyzer(logfile)
            analyzer.load_ang_data()
            analyzer.load_rate_data()
            
            ang = analyzer.data['ANG']
            rate = analyzer.data.get('RATE', None)
            
            # Plot 1: Pitch angle response
            axes[0].plot(ang['time'], ang['control_pitch'], '--', 
                        color=color, alpha=0.5, linewidth=1.5, label=f'{label} (desired)')
            axes[0].plot(ang['time'], ang['pitch'], '-', 
                        color=color, linewidth=2, label=f'{label} (actual)')
            
            # Plot 2: Pitch rate (if available)
            if rate is not None and len(rate['time']) > 0:
                axes[1].plot(rate['time'], rate['control_pitch'], '--', 
                            color=color, alpha=0.5, linewidth=1.5)
                axes[1].plot(rate['time'], rate['pitch'], '-', 
                            color=color, linewidth=2, label=label)
            
            # Plot 3: D term contribution
            axes[2].plot(ang['time'], ang['pitch_outer_D'], '-', 
                        color=color, linewidth=2, label=label)
            
            # Print metrics
            metrics = analyzer.compute_metrics()
            print(f"\n{label}:")
            print(f"  RMSE: {metrics['rmse']:.4f} deg")
            print(f"  Max Error: {metrics['max_error']:.4f} deg")
            print(f"  Max |D Term|: {metrics['max_d_term']:.4f} deg/s")
            
        except Exception as e:
            print(f"Error loading {logfile}: {e}")
            continue
    
    # Format plots
    axes[0].set_ylabel('Pitch Angle (deg)', fontsize=11)
    axes[0].set_title('Pitch Angle Response', fontsize=12)
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(loc='best', ncol=2, fontsize=9)
    
    axes[1].set_ylabel('Pitch Rate (deg/s)', fontsize=11)
    axes[1].set_title('Pitch Rate Response', fontsize=12)
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(loc='best', fontsize=9)
    
    axes[2].set_xlabel('Time (s)', fontsize=11)
    axes[2].set_ylabel('D Term Contribution (deg/s)', fontsize=11)
    axes[2].set_title('D Term Comparison', fontsize=12)
    axes[2].grid(True, alpha=0.3)
    axes[2].legend(loc='best', fontsize=9)
    axes[2].axhline(y=0, color='k', linestyle='--', alpha=0.3)
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"\nSaved comparison plot to {save_path}")
    else:
        plt.show()


def compare_sinusoid_responses(logfiles, labels=None, save_path=None):
    """Compare sinusoid tracking tests"""
    if labels is None:
        labels = [f"Test {i+1}" for i in range(len(logfiles))]
    
    fig, axes = plt.subplots(2, 1, figsize=(14, 8))
    fig.suptitle('PD Controller Sinusoid Tracking Comparison', fontsize=14, fontweight='bold')
    
    colors = plt.cm.tab10(np.linspace(0, 1, len(logfiles)))
    
    for idx, (logfile, label, color) in enumerate(zip(logfiles, labels, colors)):
        try:
            analyzer = PDLogAnalyzer(logfile)
            analyzer.load_ang_data()
            
            ang = analyzer.data['ANG']
            error = ang['pitch'] - ang['control_pitch']
            
            # Plot tracking
            axes[0].plot(ang['time'], ang['control_pitch'], '--', 
                        color=color, alpha=0.5, linewidth=1.5, label=f'{label} (desired)')
            axes[0].plot(ang['time'], ang['pitch'], '-', 
                        color=color, linewidth=2, label=f'{label} (actual)')
            
            # Plot error
            axes[1].plot(ang['time'], error, '-', 
                        color=color, linewidth=2, label=label)
            
            # Print metrics
            metrics = analyzer.compute_metrics()
            print(f"\n{label}:")
            print(f"  RMSE: {metrics['rmse']:.4f} deg")
            print(f"  Max Error: {metrics['max_error']:.4f} deg")
            
        except Exception as e:
            print(f"Error loading {logfile}: {e}")
            continue
    
    # Format plots
    axes[0].set_ylabel('Pitch Angle (deg)', fontsize=11)
    axes[0].set_title('Pitch Tracking', fontsize=12)
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(loc='best', ncol=2, fontsize=9)
    
    axes[1].set_xlabel('Time (s)', fontsize=11)
    axes[1].set_ylabel('Tracking Error (deg)', fontsize=11)
    axes[1].set_title('Tracking Error Comparison', fontsize=12)
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(loc='best', fontsize=9)
    axes[1].axhline(y=0, color='k', linestyle='--', alpha=0.3)
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"\nSaved comparison plot to {save_path}")
    else:
        plt.show()


def create_summary_table(logfiles, labels=None):
    """Create a summary table of metrics for all logs"""
    if labels is None:
        labels = [f"Test {i+1}" for i in range(len(logfiles))]
    
    print("\n" + "="*80)
    print("PD Controller Test Summary")
    print("="*80)
    print(f"{'Test':<20} {'RMSE (deg)':<15} {'Max Error (deg)':<18} {'Max |D Term|':<15}")
    print("-"*80)
    
    results = []
    for logfile, label in zip(logfiles, labels):
        try:
            analyzer = PDLogAnalyzer(logfile)
            analyzer.load_ang_data()
            metrics = analyzer.compute_metrics()
            
            results.append({
                'label': label,
                'rmse': metrics['rmse'],
                'max_error': metrics['max_error'],
                'max_d_term': metrics['max_d_term']
            })
            
            print(f"{label:<20} {metrics['rmse']:<15.4f} {metrics['max_error']:<18.4f} {metrics['max_d_term']:<15.4f}")
        except Exception as e:
            print(f"{label:<20} ERROR: {e}")
    
    print("="*80)
    return results


def main():
    parser = argparse.ArgumentParser(description='Compare PD Controller Test Logs')
    parser.add_argument('logfiles', nargs='+', help='Log file paths')
    parser.add_argument('--labels', nargs='+', help='Labels for each log file')
    parser.add_argument('--type', choices=['step', 'sinusoid', 'auto'],
                       default='auto', help='Test type')
    parser.add_argument('--save', help='Save plot to file')
    parser.add_argument('--table', action='store_true',
                       help='Print summary table')
    
    args = parser.parse_args()
    
    if len(args.logfiles) < 2:
        print("Error: Need at least 2 log files to compare")
        sys.exit(1)
    
    if args.labels and len(args.labels) != len(args.logfiles):
        print("Error: Number of labels must match number of log files")
        sys.exit(1)
    
    # Resolve log file paths
    resolved_logfiles = []
    for logfile in args.logfiles:
        logfile = os.path.expanduser(logfile)
        if not os.path.isabs(logfile):
            # Try relative to current directory, then relative to ardupilot root
            if not os.path.exists(logfile):
                ardupilot_root = os.path.join(os.path.dirname(__file__), '..', '..')
                ardupilot_root = os.path.abspath(ardupilot_root)
                alt_path = os.path.join(ardupilot_root, 'logs', os.path.basename(logfile))
                if os.path.exists(alt_path):
                    logfile = alt_path
        resolved_logfiles.append(logfile)
    
    args.logfiles = resolved_logfiles
    
    if args.table:
        create_summary_table(args.logfiles, args.labels)
    
    # Auto-detect or use specified type
    if args.type == 'auto':
        # Default to step response
        args.type = 'step'
    
    if args.type == 'step':
        compare_step_responses(args.logfiles, args.labels, args.save)
    elif args.type == 'sinusoid':
        compare_sinusoid_responses(args.logfiles, args.labels, args.save)
    
    print("\nComparison complete!")


if __name__ == '__main__':
    main()

