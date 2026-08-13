#!/usr/bin/env bash
set -euo pipefail

cd "$(cd "$(dirname "$0")" && pwd)"
[ -x build-headless/des_headless ] || ./build_headless.sh
DES=build-headless/des_headless

# $DES --config config/overrides/3_day.json
# $DES --config config/overrides/1_day.json
# $DES --config config/overrides/30_day.json
# $DES --config config/overrides/single_matrix.json
# $DES --config config/overrides/moves.json           --out-dir results/moves
$DES --config config/overrides/moves_recognize.json --out-dir results/moves_recognize

# $DES --base-config bench/baseline_config.json --config config/overrides/F1/2_beta_smoothed.json      --out-dir results/F1_beta_smoothed
# $DES --base-config bench/baseline_config.json --config config/overrides/F2/3_arbeitsplatz_rolle.json --out-dir results/F2_arbeitsplatz_rolle
# $DES --base-config bench/baseline_config.json --config config/overrides/F3/2_cost_aware.json         --out-dir results/F3_cost_aware
# $DES --base-config bench/baseline_config.json --config config/overrides/F4/3_mit_absuche.json        --out-dir results/F4_mit_absuche
# $DES --base-config bench/baseline_config.json --config config/overrides/F5/4_re_8_auskunft.json      --out-dir results/F5_auskunft
