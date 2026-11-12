#!/usr/bin/env bash
set -euo pipefail
here=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
first_trial=${1:-0}
[[ "$first_trial" =~ ^[0-9]+$ ]] || exit 2
for ((trial=first_trial;trial<first_trial+3;trial++));do
 for world in 0 6 12;do
  for method in mppi slash;do "$here/run.sh" "$method" "$world" "$trial";done
 done
done
