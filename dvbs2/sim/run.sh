#!/bin/sh
# Run a cocotb testbench with the project venv on PATH.
#   ./run.sh                          all testbenches
#   ./run.sh dvbs2_fec test_fec       one testbench
cd "$(dirname "$0")"
export PATH="$(cd .. && pwd)/.venv/bin:$PATH"
if [ $# -ge 2 ]; then
	t=$1; mod=$2; shift 2; exec make TOPLEVEL="$t" MODULE="$mod" "$@"
fi
exec make all-tests
