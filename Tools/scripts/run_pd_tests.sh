#!/bin/bash
# Simple wrapper script to run PD controller tests and analyze results

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ARDUPILOT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
LOGS_DIR="$ARDUPILOT_ROOT/logs"

echo "=========================================="
echo "PD Controller Test Suite"
echo "=========================================="
echo ""

# Check if SITL is running
if ! pgrep -f "sim_vehicle.py" > /dev/null && ! pgrep -f "ArduCopter" > /dev/null; then
    echo "WARNING: SITL doesn't appear to be running!"
    echo "Start it with: ./Tools/autotest/sim_vehicle.py -v ArduCopter --console --map"
    echo ""
    read -p "Continue anyway? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Run tests
echo "Running PD controller tests..."
echo ""
cd "$SCRIPT_DIR"
python3 test_pd_controller.py

echo ""
echo "=========================================="
echo "Tests Complete!"
echo "=========================================="
echo ""

# Find latest logs
echo "Finding latest log files..."
LATEST_LOGS=$(python3 find_latest_logs.py -n 3 --paths 2>/dev/null | grep -E "^[0-9]+\." | awk '{print $2}')

if [ -z "$LATEST_LOGS" ]; then
    echo "Could not find log files automatically."
    echo "Logs should be in: $LOGS_DIR"
    echo ""
    echo "To analyze manually:"
    echo "  python3 plot_pd_results.py $LOGS_DIR/0000000X.BIN --type step --metrics"
    exit 0
fi

# Convert to array
readarray -t LOG_ARRAY <<< "$LATEST_LOGS"

if [ ${#LOG_ARRAY[@]} -ge 3 ]; then
    echo "Found 3 latest logs:"
    for i in "${!LOG_ARRAY[@]}"; do
        echo "  $((i+1)). $(basename "${LOG_ARRAY[$i]}")"
    done
    echo ""
    echo "To analyze individual logs:"
    echo "  python3 plot_pd_results.py ${LOG_ARRAY[0]} --type step --metrics"
    echo ""
    echo "To compare all 3 tests:"
    echo "  python3 compare_pd_logs.py \\"
    echo "    ${LOG_ARRAY[0]} \\"
    echo "    ${LOG_ARRAY[1]} \\"
    echo "    ${LOG_ARRAY[2]} \\"
    echo "    --labels 'KD=0' 'KD=Low' 'KD=High' --save comparison.png --table"
    echo ""
    read -p "Run comparison now? (y/n) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        python3 compare_pd_logs.py \
            "${LOG_ARRAY[0]}" \
            "${LOG_ARRAY[1]}" \
            "${LOG_ARRAY[2]}" \
            --labels "KD=0" "KD=Low" "KD=High" \
            --save "$LOGS_DIR/comparison.png" \
            --table
        echo ""
        echo "Comparison saved to: $LOGS_DIR/comparison.png"
    fi
else
    echo "Found ${#LOG_ARRAY[@]} log(s):"
    for log in "${LOG_ARRAY[@]}"; do
        echo "  - $(basename "$log")"
    done
    echo ""
    echo "To analyze:"
    echo "  python3 plot_pd_results.py ${LOG_ARRAY[0]} --type step --metrics"
fi

