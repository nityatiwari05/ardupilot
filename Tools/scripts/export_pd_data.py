#!/usr/bin/env python3
"""
Export PD controller log data to CSV for external analysis
"""

import sys
import os
import csv
import argparse
from plot_pd_results import PDLogAnalyzer


def export_to_csv(analyzer, output_file):
    """Export ANG and RATE data to CSV"""
    if 'ANG' not in analyzer.data:
        analyzer.load_ang_data()
    
    if 'RATE' not in analyzer.data:
        analyzer.load_rate_data()
    
    ang = analyzer.data['ANG']
    rate = analyzer.data.get('RATE', None)
    
    # Prepare data
    data_rows = []
    
    # Use ANG time as primary timeline
    for i in range(len(ang['time'])):
        row = {
            'time': ang['time'][i],
            'desired_pitch': ang['control_pitch'][i],
            'actual_pitch': ang['pitch'][i],
            'pitch_error': ang['pitch'][i] - ang['control_pitch'][i],
            'pitch_outer_D': ang['pitch_outer_D'][i],
            'pitch_deriv_error': ang['pitch_deriv_error'][i],
            'sensor_dt': ang['sensor_dt'][i],
        }
        
        # Add rate data if available (interpolate if needed)
        if rate is not None and len(rate['time']) > 0:
            # Find closest rate data point
            rate_idx = min(range(len(rate['time'])), 
                          key=lambda j: abs(rate['time'][j] - ang['time'][i]))
            if abs(rate['time'][rate_idx] - ang['time'][i]) < 0.1:  # Within 100ms
                row['desired_pitch_rate'] = rate['control_pitch'][rate_idx]
                row['actual_pitch_rate'] = rate['pitch'][rate_idx]
                row['pitch_rate_out'] = rate['pitch_out'][rate_idx]
            else:
                row['desired_pitch_rate'] = None
                row['actual_pitch_rate'] = None
                row['pitch_rate_out'] = None
        else:
            row['desired_pitch_rate'] = None
            row['actual_pitch_rate'] = None
            row['pitch_rate_out'] = None
        
        data_rows.append(row)
    
    # Write to CSV
    fieldnames = ['time', 'desired_pitch', 'actual_pitch', 'pitch_error',
                  'desired_pitch_rate', 'actual_pitch_rate', 'pitch_rate_out',
                  'pitch_outer_D', 'pitch_deriv_error', 'sensor_dt']
    
    with open(output_file, 'w', newline='') as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()
        for row in data_rows:
            # Convert None to empty string for CSV
            clean_row = {k: (v if v is not None else '') for k, v in row.items()}
            writer.writerow(clean_row)
    
    print(f"Exported {len(data_rows)} data points to {output_file}")


def main():
    parser = argparse.ArgumentParser(description='Export PD Controller log data to CSV')
    parser.add_argument('logfile', help='Log file path')
    parser.add_argument('-o', '--output', help='Output CSV file (default: logfile.csv)')
    
    args = parser.parse_args()
    
    # Resolve log file path
    logfile = os.path.expanduser(args.logfile)
    if not os.path.isabs(logfile):
        if not os.path.exists(logfile):
            ardupilot_root = os.path.join(os.path.dirname(__file__), '..', '..')
            ardupilot_root = os.path.abspath(ardupilot_root)
            alt_path = os.path.join(ardupilot_root, 'logs', os.path.basename(logfile))
            if os.path.exists(alt_path):
                logfile = alt_path
    
    if not os.path.exists(logfile):
        print(f"Error: Log file not found: {args.logfile}")
        sys.exit(1)
    
    args.logfile = logfile
    
    if args.output is None:
        args.output = logfile.replace('.BIN', '.csv').replace('.tlog', '.csv')
    
    analyzer = PDLogAnalyzer(args.logfile)
    export_to_csv(analyzer, args.output)
    
    print(f"\nCSV file ready: {args.output}")
    print("You can now analyze this data in Excel, MATLAB, or other tools")


if __name__ == '__main__':
    main()

