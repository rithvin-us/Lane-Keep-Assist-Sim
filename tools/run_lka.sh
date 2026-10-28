#!/usr/bin/env bash
# Runs the LKA without opening the MATLAB window (terminal, cron).
#   tools/run_lka.sh          every video in vids/  -> test_res/<video>/ and test_res/summary.csv
#   tools/run_lka.sh watch    process new videos in vids/ once (put this in cron)
#   tools/run_lka.sh checks   quick checks: all unit tests -> results/checks/lka_checks_<time>.md
#   tools/run_lka.sh full     full checks: + closed-loop MIL tests + every video (straight-lane alarm)
set -euo pipefail
cd "$(dirname "$0")/.."
case "${1:-videos}" in
  videos) cmd="addpath('scripts'); run_lka_on_test_videos" ;;
  watch)  cmd="addpath('scripts'); watch_lka_videos(Once=true)" ;;
  checks) cmd="addpath('scripts'); run_lka_checks" ;;
  full)   cmd="addpath('scripts'); run_lka_checks(Level='full')" ;;
  *) echo "usage: $0 [videos|watch|checks|full]" >&2; exit 2 ;;
esac
echo "Running: matlab -batch \"$cmd\""
exec matlab -batch "$cmd"
