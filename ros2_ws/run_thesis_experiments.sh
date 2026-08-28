#!/usr/bin/env bash

# BEGIN AI-assisted: generated with Claude Code (Anthropic),
# reviewed and adapted by the author.
#
# Runs every experiment of the thesis.
#
#     ./run_thesis_experiments.sh          20 rounds, as used in the thesis
#     ./run_thesis_experiments.sh 2        quick test
#
# One line is one simulation run. The arms of an experiment run in parallel,
# the experiments run one after another. Results end up in
# results/thesis/<experiment>/runs/<arm>/metrics.csv
#
# All arms share the baseline of scenarios/thesis_baseline.json: 84 people,
# 12 appointments a day and 30 simulated days, as in the paper. Generate the
# scenarios with tools/gen_thesis_scenario.py before the first run.
#
# Every experiment also has a gui.json next to its arms. It is a shortened
# single arm meant for ./full.sh, not for this script.
#
set -uo pipefail
cd "$(dirname "$0")"

ROUNDS=${1:-20}
BIN=build-headless/des_headless
FAILED=results/thesis/.failed

if [ ! -x "$BIN" ]; then
    ./build_headless.sh
fi
rm -rf results/thesis
mkdir -p results/thesis
echo "results/thesis geleert"

run() {
    local dir="results/thesis/$1/runs/$2"
    mkdir -p "$dir"
    (
        if ! "$BIN" --mode headless --config "config/overrides/thesis/$3" --out-dir "$dir" --rounds "$ROUNDS" --log-level des:=WARN > "$dir/sim.log" 2>&1; then
            echo "  FAILED  $1/$2  see $dir/sim.log"
            touch "$FAILED"
        elif grep -q "SoC 0" "$dir/sim.log"; then
            echo "  FAILED  $1/$2  battery empty, see $dir/sim.log"
            touch "$FAILED"
        else
            echo "  done    $1/$2"
        fi
    ) &
}

barrier() {
    wait
    if [ -f "$FAILED" ]; then
        echo "aborted"
        exit 1
    fi
}

echo "$ROUNDS rounds per arm"

echo "T1   Energiereserve, 44 Ah und 235 W"
run T1_energiereserve next_mission      T1_energiereserve/1_next_mission.json
run T1_energiereserve horizon_01h       T1_energiereserve/2_horizon_01h.json
run T1_energiereserve horizon_02h       T1_energiereserve/3_horizon_02h.json
run T1_energiereserve horizon_04h       T1_energiereserve/4_horizon_04h.json
run T1_energiereserve horizon_12h       T1_energiereserve/8_horizon_12h.json
barrier

echo "T1b  dasselbe mit abgeschaltetem Roboter am Dock, 44 Ah und 385 W"
run T1b_ladegeraet_doppelt next_mission      T1b_ladegeraet_doppelt/1_next_mission.json
run T1b_ladegeraet_doppelt horizon_01h       T1b_ladegeraet_doppelt/2_horizon_01h.json
run T1b_ladegeraet_doppelt horizon_02h       T1b_ladegeraet_doppelt/3_horizon_02h.json
run T1b_ladegeraet_doppelt horizon_04h       T1b_ladegeraet_doppelt/4_horizon_04h.json
run T1b_ladegeraet_doppelt horizon_12h       T1b_ladegeraet_doppelt/8_horizon_12h.json
barrier

echo "T1c  dasselbe mit groesserer Batterie, 66 Ah und 235 W"
run T1c_akku_doppelt next_mission      T1c_akku_doppelt/1_next_mission.json
run T1c_akku_doppelt horizon_01h       T1c_akku_doppelt/2_horizon_01h.json
run T1c_akku_doppelt horizon_02h       T1c_akku_doppelt/3_horizon_02h.json
run T1c_akku_doppelt horizon_04h       T1c_akku_doppelt/4_horizon_04h.json
run T1c_akku_doppelt horizon_12h       T1c_akku_doppelt/8_horizon_12h.json
barrier

echo "T1d  dasselbe mit beidem, 66 Ah und 385 W"
run T1d_beides_doppelt next_mission      T1d_beides_doppelt/1_next_mission.json
run T1d_beides_doppelt horizon_01h       T1d_beides_doppelt/2_horizon_01h.json
run T1d_beides_doppelt horizon_02h       T1d_beides_doppelt/3_horizon_02h.json
run T1d_beides_doppelt horizon_04h       T1d_beides_doppelt/4_horizon_04h.json
run T1d_beides_doppelt horizon_12h       T1d_beides_doppelt/8_horizon_12h.json
barrier

echo "T11  Anfragelast, wie viele Unterbrechungen vertraegt der Roboter"
run T11_anfragelast rate_02           T11_anfragelast/1_rate_02.json
run T11_anfragelast rate_04           T11_anfragelast/2_rate_04.json
run T11_anfragelast rate_08           T11_anfragelast/3_rate_08.json
run T11_anfragelast rate_16           T11_anfragelast/4_rate_16.json
run T11_anfragelast rate_24           T11_anfragelast/5_rate_24.json
run T11_anfragelast rate_32           T11_anfragelast/6_rate_32.json
barrier

echo "T4   Lastgrenze, ab welcher Termindichte lehnt der Roboter ab"
run T4_lastgrenze last_12             T4_lastgrenze/1_last_12.json
run T4_lastgrenze last_18             T4_lastgrenze/2_last_18.json
run T4_lastgrenze last_24             T4_lastgrenze/3_last_24.json
run T4_lastgrenze last_36             T4_lastgrenze/4_last_36.json
run T4_lastgrenze last_48             T4_lastgrenze/5_last_48.json
run T4_lastgrenze last_60             T4_lastgrenze/6_last_60.json
barrier

echo "T16  Termintreue, Vorsprung und Verspaetung ueber die Simulationstage"
run T16_termintreue baseline          T16_termintreue/1_baseline.json
barrier

echo "T17  Batteriereserve, Anteil der Ladung, den der Roboter zurueckhaelt"
run T17_batteriereserve reserve_10        T17_batteriereserve/1_reserve_10.json
run T17_batteriereserve reserve_15        T17_batteriereserve/2_reserve_15.json
run T17_batteriereserve reserve_20        T17_batteriereserve/3_reserve_20.json
run T17_batteriereserve reserve_25        T17_batteriereserve/4_reserve_25.json
run T17_batteriereserve reserve_30        T17_batteriereserve/5_reserve_30.json
run T17_batteriereserve reserve_40        T17_batteriereserve/6_reserve_40.json
run T17_batteriereserve reserve_50        T17_batteriereserve/7_reserve_50.json
barrier

echo "T14  Routenplanung, Kostenteiler und GRASP-Parameter"
run T14_orienteering standard          T14_orienteering/01_standard.json
run T14_orienteering ohne_kosten       T14_orienteering/02_ohne_kosten.json
run T14_orienteering iter_001          T14_orienteering/03_iter_001.json
run T14_orienteering iter_010          T14_orienteering/04_iter_010.json
run T14_orienteering iter_050          T14_orienteering/05_iter_050.json
run T14_orienteering iter_500          T14_orienteering/06_iter_500.json
run T14_orienteering alpha_000         T14_orienteering/07_alpha_000.json
run T14_orienteering alpha_020         T14_orienteering/09_alpha_020.json
run T14_orienteering alpha_050         T14_orienteering/12_alpha_050.json
run T14_orienteering alpha_080         T14_orienteering/15_alpha_080.json
run T14_orienteering alpha_100         T14_orienteering/17_alpha_100.json
barrier

echo "T14b dasselbe ohne Termine, nur Hintergrundmissionen"
run T14b_ohne_termine standard          T14b_ohne_termine/01_standard.json
run T14b_ohne_termine ohne_kosten       T14b_ohne_termine/02_ohne_kosten.json
run T14b_ohne_termine iter_001          T14b_ohne_termine/03_iter_001.json
run T14b_ohne_termine iter_010          T14b_ohne_termine/04_iter_010.json
run T14b_ohne_termine iter_050          T14b_ohne_termine/05_iter_050.json
run T14b_ohne_termine iter_500          T14b_ohne_termine/06_iter_500.json
run T14b_ohne_termine alpha_000         T14b_ohne_termine/07_alpha_000.json
run T14b_ohne_termine alpha_020         T14b_ohne_termine/09_alpha_020.json
run T14b_ohne_termine alpha_050         T14b_ohne_termine/12_alpha_050.json
run T14b_ohne_termine alpha_080         T14b_ohne_termine/15_alpha_080.json
run T14b_ohne_termine alpha_100         T14b_ohne_termine/17_alpha_100.json
barrier

echo "T14h Wirksamkeit der Routenplanung, kurze Auftraege und dominierende Fahrzeit"
run T14h_fahrt_dominiert standard_m10           T14h_fahrt_dominiert/02_standard_m10.json
run T14h_fahrt_dominiert standard_m15           T14h_fahrt_dominiert/03_standard_m15.json
run T14h_fahrt_dominiert standard_m20           T14h_fahrt_dominiert/04_standard_m20.json
run T14h_fahrt_dominiert ohne_kosten_m10        T14h_fahrt_dominiert/07_ohne_kosten_m10.json
run T14h_fahrt_dominiert ohne_kosten_m15        T14h_fahrt_dominiert/08_ohne_kosten_m15.json
run T14h_fahrt_dominiert ohne_kosten_m20        T14h_fahrt_dominiert/09_ohne_kosten_m20.json
run T14h_fahrt_dominiert iter_001_m10           T14h_fahrt_dominiert/12_iter_001_m10.json
run T14h_fahrt_dominiert iter_001_m15           T14h_fahrt_dominiert/13_iter_001_m15.json
run T14h_fahrt_dominiert iter_001_m20           T14h_fahrt_dominiert/14_iter_001_m20.json
run T14h_fahrt_dominiert iter_010_m10           T14h_fahrt_dominiert/17_iter_010_m10.json
run T14h_fahrt_dominiert iter_010_m15           T14h_fahrt_dominiert/18_iter_010_m15.json
run T14h_fahrt_dominiert iter_010_m20           T14h_fahrt_dominiert/19_iter_010_m20.json
run T14h_fahrt_dominiert iter_050_m10           T14h_fahrt_dominiert/22_iter_050_m10.json
run T14h_fahrt_dominiert iter_050_m15           T14h_fahrt_dominiert/23_iter_050_m15.json
run T14h_fahrt_dominiert iter_050_m20           T14h_fahrt_dominiert/24_iter_050_m20.json
run T14h_fahrt_dominiert iter_500_m10           T14h_fahrt_dominiert/27_iter_500_m10.json
run T14h_fahrt_dominiert iter_500_m15           T14h_fahrt_dominiert/28_iter_500_m15.json
run T14h_fahrt_dominiert iter_500_m20           T14h_fahrt_dominiert/29_iter_500_m20.json
run T14h_fahrt_dominiert alpha_000_m10          T14h_fahrt_dominiert/32_alpha_000_m10.json
run T14h_fahrt_dominiert alpha_000_m15          T14h_fahrt_dominiert/33_alpha_000_m15.json
run T14h_fahrt_dominiert alpha_000_m20          T14h_fahrt_dominiert/34_alpha_000_m20.json
run T14h_fahrt_dominiert alpha_020_m10          T14h_fahrt_dominiert/37_alpha_020_m10.json
run T14h_fahrt_dominiert alpha_020_m15          T14h_fahrt_dominiert/38_alpha_020_m15.json
run T14h_fahrt_dominiert alpha_020_m20          T14h_fahrt_dominiert/39_alpha_020_m20.json
run T14h_fahrt_dominiert alpha_050_m10          T14h_fahrt_dominiert/42_alpha_050_m10.json
run T14h_fahrt_dominiert alpha_050_m15          T14h_fahrt_dominiert/43_alpha_050_m15.json
run T14h_fahrt_dominiert alpha_050_m20          T14h_fahrt_dominiert/44_alpha_050_m20.json
run T14h_fahrt_dominiert alpha_080_m10          T14h_fahrt_dominiert/47_alpha_080_m10.json
run T14h_fahrt_dominiert alpha_080_m15          T14h_fahrt_dominiert/48_alpha_080_m15.json
run T14h_fahrt_dominiert alpha_080_m20          T14h_fahrt_dominiert/49_alpha_080_m20.json
run T14h_fahrt_dominiert alpha_100_m10          T14h_fahrt_dominiert/52_alpha_100_m10.json
run T14h_fahrt_dominiert alpha_100_m15          T14h_fahrt_dominiert/53_alpha_100_m15.json
run T14h_fahrt_dominiert alpha_100_m20          T14h_fahrt_dominiert/54_alpha_100_m20.json
barrier

echo "T15  Nutzengewichte, Verhaeltnis Akquise zu Reinigung"
run T15_nutzenfunktion wert_000       T15_nutzenfunktion/01_wert_000.json
run T15_nutzenfunktion wert_006       T15_nutzenfunktion/02_wert_006.json
run T15_nutzenfunktion wert_015       T15_nutzenfunktion/03_wert_015.json
run T15_nutzenfunktion wert_024       T15_nutzenfunktion/04_wert_024.json
run T15_nutzenfunktion wert_030       T15_nutzenfunktion/05_wert_030.json
run T15_nutzenfunktion wert_045       T15_nutzenfunktion/07_wert_045.json
run T15_nutzenfunktion wert_090       T15_nutzenfunktion/09_wert_090.json
run T15_nutzenfunktion wert_300       T15_nutzenfunktion/11_wert_300.json
barrier

echo
if [ -f "$FAILED" ]; then
    echo "some runs failed"
    exit 1
fi
echo "all experiments done, results in results/thesis/"
