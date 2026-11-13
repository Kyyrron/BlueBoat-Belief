#!/usr/bin/env bash
# The BlueBoat-Belief offline gate, run in the two Python environments that matter on the
# operator laptop (no ROS needed, nothing written outside a temporary directory):
#
#   A. the project venv   ~/ros2_ws/.venv   (numpy 2.x, rosbags)
#   B. the system python  /usr/bin/python3  (numpy 1.26, the one ROS nodes import),
#      with the venv's site-packages appended LAST, so only `rosbags` is taken from it.
#
# Both must print "ALL CHECKS PASSED" with 0 skipped. Usage:  test/gate.sh [-v]
#   -v  print every check (default: section headers, failures, skips and the summary)
#
# Do not source ~/ros2_ws/install/setup.bash first: the gate is the ROS-free one.
set -u
cd "$(dirname "$0")/.." || exit 2
VENV="${BLUEBOAT_VENV:-$HOME/ros2_ws/.venv}"
SITE="$(ls -d "$VENV"/lib/python3*/site-packages 2>/dev/null | head -1)"
FILTER='^\[[0-9]+\]|FAIL|SKIP|ALL CHECKS|failed at|Traceback|Error'
[ "${1:-}" = "-v" ] && FILTER='.'
export PYTHONDONTWRITEBYTECODE=1
unset PYTHONPATH COLCON_PREFIX_PATH AMENT_PREFIX_PATH
rc=0

echo "=== A: $VENV/bin/python3 ==="
if [ -x "$VENV/bin/python3" ]; then
    "$VENV/bin/python3" -m test.smoke_test 2>&1 | grep -E "$FILTER"
    [ "${PIPESTATUS[0]}" -eq 0 ] || rc=1
else
    echo "  SKIP: no venv at $VENV (set BLUEBOAT_VENV)"; rc=1
fi

echo "=== B: /usr/bin/python3 (system numpy; rosbags from the venv, appended last) ==="
/usr/bin/python3 -c "
import runpy, sys
site = '$SITE'
if site:
    sys.path.append(site)
import numpy
print('  numpy', numpy.__version__, 'from', numpy.__file__.rsplit('/numpy/', 1)[0])
runpy.run_module('test.smoke_test', run_name='__main__')
" 2>&1 | grep -E "$FILTER|^  numpy"
[ "${PIPESTATUS[0]}" -eq 0 ] || rc=1

exit $rc
