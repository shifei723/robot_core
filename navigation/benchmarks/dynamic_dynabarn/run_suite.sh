#!/usr/bin/env bash
set -euo pipefail
here=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
trial=${1:-0}
[[ "$trial" =~ ^[0-9]+$ ]] || exit 2
for phase in 0 7 14;do
 if [[ "$phase" == 0 ]];then methods=(mppi frs_soft frs_joint)
 elif [[ "$phase" == 7 ]];then methods=(frs_joint mppi frs_soft)
 else methods=(frs_soft frs_joint mppi);fi
 for method in "${methods[@]}";do "$here/run.sh" "$method" "$phase" "$trial";done
done
