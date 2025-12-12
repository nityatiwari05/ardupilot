# PD Controller SITL Testing Guide

This guide explains how to test the Pitch Angle PD Controller implementation in ArduPilot SITL.

## Prerequisites

1. **ArduPilot SITL** running and accessible via MAVLink
2. **Python packages**:
   ```bash
   pip3 install pymavlink matplotlib numpy
   ```
3. **MAVProxy** or direct MAVLink connection to SITL

## Quick Start

### 1. Start SITL

```bash
cd ~/ardupilot
./Tools/autotest/sim_vehicle.py -v ArduCopter --console --map
```

Wait for SITL to initialize and connect.

### 2. Run Automated Tests

The test script will run all test cases automatically:

```bash
cd ~/ardupilot/Tools/scripts
python3 test_pd_controller.py
```

Or run specific test types:

```bash
# Only step response tests
python3 test_pd_controller.py --test step

# Only sinusoid tests
python3 test_pd_controller.py --test sinusoid
```

### 3. Analyze Results

After tests complete, download logs from SITL and analyze:

```bash
# Analyze a single log file
python3 plot_pd_results.py /path/to/log.BIN --type step --metrics

# Save plot to file
python3 plot_pd_results.py /path/to/log.BIN --type step --save plot.png
```

## Manual Testing Procedure

If you prefer manual testing, follow these steps:

### A. Baseline Check (KD=0)

1. **Set parameters**:
   ```
   param set ATC_ANG_PIT_D 0
   param set ATC_ANG_PIT_DF 15
   param save
   ```

2. **Arm and takeoff**:
   ```
   mode GUIDED
   arm throttle
   takeoff 5
   ```

3. **Command pitch step**:
   - Option 1: Use RC override (if RC channel 2 is pitch):
     ```
     rc 2 1600  # Adjust value based on your mapping
     ```
   - Option 2: Use SET_ATTITUDE_TARGET MAVLink message
   - Option 3: Use Guided mode attitude setpoint in Mission Planner/QGroundControl

4. **Wait for response** (15-20 seconds)

5. **Return to level and land**:
   ```
   rc 2 1500  # Center stick
   mode LAND
   ```

6. **Download log** from SITL (usually in `logs/` directory)

### B. Low KD Test

Repeat steps A.1-A.6 with:
```
param set ATC_ANG_PIT_D 0.05
```

### C. Medium KD Test

Repeat with:
```
param set ATC_ANG_PIT_D 0.2
```

### D. High KD Test

Repeat with:
```
param set ATC_ANG_PIT_D 1.0
```

**Note**: High KD may show instability or noise amplification. Monitor for:
- Oscillations
- Excessive noise in derivative term
- Poor settling behavior

### E. Sinusoid Tracking Tests

For each KD value (0.0, 0.2, 1.0) and frequency (0.2 Hz, 0.5 Hz, 1.0 Hz):

1. Set KD parameter
2. Arm and takeoff
3. Command sinusoidal pitch: `theta_c = A*sin(2*pi*f*t)`
   - Amplitude A = 5°
   - Duration = 20 seconds

You can use the automated script for sinusoid tests:
```bash
python3 test_pd_controller.py --test sinusoid
```

## Plotting and Analysis

### Single Log Analysis

```bash
python3 plot_pd_results.py logs/00000001.BIN --type step --metrics
```

This will:
- Load ANG and RATE messages from the log
- Plot pitch response, rate response, and PD terms
- Print performance metrics (RMSE, max error, etc.)

### Compare Multiple Logs

Create a Python script to compare:

```python
from plot_pd_results import compare_logs

compare_logs(
    ['logs/baseline.BIN', 'logs/kd_0.2.BIN', 'logs/kd_1.0.BIN'],
    labels=['KD=0', 'KD=0.2', 'KD=1.0'],
    test_type='step'
)
```

### Expected Plots

For **step response tests**, you should see:
1. **Pitch Angle Response**: Desired vs actual pitch angle
2. **Pitch Rate Response**: Desired vs actual pitch rate
3. **PD Terms**: D-term contribution and derivative error

For **sinusoid tests**, you should see:
1. **Tracking Performance**: How well actual pitch follows desired
2. **Tracking Error**: Error vs time
3. **Frequency Spectrum**: FFT of tracking error

## Performance Metrics

The analysis script computes:
- **RMSE**: Root Mean Square Error (deg)
- **Max Error**: Maximum absolute error (deg)
- **Mean |D Term|**: Average magnitude of D-term contribution
- **Max |D Term|**: Peak D-term contribution
- **Mean |Deriv Error|**: Average derivative error magnitude
- **Max |Deriv Error|**: Peak derivative error

## Key Observations

### Baseline (KD=0)
- Should behave like pure P controller
- No derivative term contribution
- May show overshoot and slower settling

### Low KD (0.05)
- Small derivative contribution
- Reduced overshoot
- Faster settling than baseline

### Medium KD (0.2)
- Moderate derivative contribution
- Good damping
- Fast settling without overshoot

### High KD (1.0)
- Large derivative contribution
- May show:
  - Noise amplification (if filter insufficient)
  - Over-damping (slower response)
  - Instability (if too high)

## Troubleshooting

### Connection Issues
- Ensure SITL is running: `./Tools/autotest/sim_vehicle.py -v ArduCopter`
- Check MAVLink connection: Default is `udp:127.0.0.1:14550`
- Verify with: `mavproxy.py --master=udp:127.0.0.1:14550`

### Log File Issues
- Logs are typically in `logs/` directory
- Look for `.BIN` files (binary format)
- Use `mavlogdump.py` to convert if needed

### Plotting Issues
- Ensure matplotlib is installed: `pip3 install matplotlib`
- For headless systems, use: `export MPLBACKEND=Agg`

## Advanced Analysis

### Derivative Kick Analysis
Check that derivative term doesn't spike on setpoint changes:
- Plot `pitch_deriv_error` vs time
- Should be smooth, no sudden jumps when setpoint changes
- This confirms derivative-on-measurement is working

### Noise Analysis
Check derivative error for noise:
- Plot `pitch_deriv_error` spectrum (FFT)
- Verify filter cutoff (ANG_PIT_DF) is appropriate
- Higher cutoff = faster response but more noise
- Lower cutoff = smoother but slower response

### Filter Design Validation
- Use ArduPilot Filter Review tool if available
- Analyze gyro noise characteristics
- Verify PT1 filter (ANG_PIT_DF) provides adequate filtering

## References

- ArduPilot Logging: https://ardupilot.org/dev/docs/logs.html
- MAVLink Documentation: https://mavlink.io/
- PD Controller Theory: Standard control systems textbooks

