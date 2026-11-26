#!/usr/bin/env bash
set -eo pipefail
# Same dedicated serial/domain and detector; explicit replacement of VINS, no duplicate TF.
export LOCALIZATION_MODE=openvins
exec bash "$(dirname -- "${BASH_SOURCE[0]}")/run_pipeline.sh" "$@"
