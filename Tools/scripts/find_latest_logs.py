#!/usr/bin/env python3
"""
Find the latest log files in the ArduPilot logs directory
"""

import os
import glob
import sys

def find_latest_logs(count=3, logs_dir=None):
    """Find the N most recent log files"""
    if logs_dir is None:
        # Try to find ardupilot root
        script_dir = os.path.dirname(os.path.abspath(__file__))
        ardupilot_root = os.path.join(script_dir, '..', '..')
        ardupilot_root = os.path.abspath(ardupilot_root)
        logs_dir = os.path.join(ardupilot_root, 'logs')
    else:
        logs_dir = os.path.expanduser(logs_dir)
    
    if not os.path.exists(logs_dir):
        print(f"Error: Logs directory not found: {logs_dir}")
        return []
    
    # Find all .BIN files
    pattern = os.path.join(logs_dir, '*.BIN')
    log_files = glob.glob(pattern)
    
    # Sort by modification time (newest first)
    log_files.sort(key=os.path.getmtime, reverse=True)
    
    return log_files[:count]

def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='Find latest ArduPilot log files')
    parser.add_argument('-n', '--count', type=int, default=3,
                       help='Number of latest logs to find (default: 3)')
    parser.add_argument('--dir', help='Logs directory (default: ~/ardupilot/logs)')
    parser.add_argument('--paths', action='store_true',
                       help='Print full paths instead of just filenames')
    
    args = parser.parse_args()
    
    logs = find_latest_logs(args.count, args.dir)
    
    if not logs:
        print("No log files found!")
        sys.exit(1)
    
    print(f"Found {len(logs)} latest log file(s):\n")
    for i, logfile in enumerate(logs, 1):
        if args.paths:
            print(f"{i}. {logfile}")
        else:
            print(f"{i}. {os.path.basename(logfile)}")
    
    print("\nTo analyze:")
    if args.paths:
        log_paths = ' '.join(logs)
    else:
        log_paths = ' '.join([os.path.basename(f) for f in logs])
    print(f"  python3 plot_pd_results.py {logs[0]} --type step --metrics")
    
    if len(logs) >= 3:
        print(f"\nTo compare all 3:")
        print(f"  python3 compare_pd_logs.py {' '.join(logs)} \\")
        print(f"    --labels 'KD=0' 'KD=Low' 'KD=High' --save comparison.png --table")

if __name__ == '__main__':
    main()

