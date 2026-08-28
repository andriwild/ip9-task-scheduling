#!/usr/bin/env bash

# BEGIN AI-assisted: generated with Claude Code (Anthropic),
# reviewed and adapted by the author.
#
# Runs every experiment of the paper.
#
#     ./run_paper_experiments.sh          20 rounds, as used in the paper
#     ./run_paper_experiments.sh 2        quick test
#
# One line is one simulation run. The arms of an experiment run in parallel,
# the experiments run one after another. Results end up in
# results/paper/<experiment>/runs/<arm>/metrics_daily.csv
#
# Afterwards: python3 tools/paper_csv.py
#
set -uo pipefail
cd "$(dirname "$0")"

ROUNDS=${1:-20}
BIN=build-headless/des_headless
FAILED=results/paper/.failed

if [ ! -x "$BIN" ]; then
    ./build_headless.sh
fi
mkdir -p results/paper
rm -f "$FAILED"

run() {
    local dir="results/paper/$1/runs/$2"
    mkdir -p "$dir"
    (
        if ! "$BIN" --mode headless --config "config/overrides/paper/$3" --out-dir "$dir" --rounds "$ROUNDS" --log-level des:=WARN > "$dir/sim.log" 2>&1; then
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

# Has to run first: it measures where the people are, and the search oracle of
# F1 reads that measurement. One year, one seed, no orders, so the robot cannot
# move anybody. tools/occupancy_csv.py turns the trace into the oracle.
echo "P1  occupancy over one year without orders"
mkdir -p results/paper/P1_aufenthalt/runs/messung
if "$BIN" --mode headless --config config/overrides/paper/P1_aufenthalt/1_messung.json \
        --out-dir results/paper/P1_aufenthalt/runs/messung --rounds 1 --log-level des:=WARN \
        > results/paper/P1_aufenthalt/runs/messung/sim.log 2>&1; then
    python3 tools/occupancy_csv.py > results/paper/P1_aufenthalt/occupancy.log
    echo "  done    P1/messung, oracle written to config/default/sim_config.json"
else
    echo "  FAILED  P1/messung  see results/paper/P1_aufenthalt/runs/messung/sim.log"
    exit 1
fi

echo "F1  room value"
run F1 frequency          F1/1_frequency.json
run F1 beta_smoothed      F1/2_beta_smoothed.json
run F1 gleichverteilt     F1/3_gleichverteilt.json
run F1 wahre_verteilung   F1/4_wahre_verteilung.json
barrier

echo "F1_kurz  room value at re = 2.5 m"
run F1_kurz frequency      F1_kurz/1_frequency.json
run F1_kurz beta_smoothed  F1_kurz/2_beta_smoothed.json
run F1_kurz gleichverteilt F1_kurz/3_gleichverteilt.json
run F1_kurz wahre_verteilung F1_kurz/4_wahre_verteilung.json
barrier

echo "F2  learning curve"
run F2 ohne_vorwissen     F2/1_ohne_vorwissen.json
run F2 arbeitsplatz       F2/2_arbeitsplatz.json
barrier

echo "F3  cost aware room order"
run F3 probability_only   F3/1_probability_only.json
run F3 cost_aware         F3/2_cost_aware.json
barrier

echo "F3_kurz  cost aware room order at re = 2.5 m"
run F3_kurz probability_only F3_kurz/1_probability_only.json
run F3_kurz cost_aware       F3_kurz/2_cost_aware.json
barrier

echo "F5  recognition range"
run F5 re_2               F5/1_re_2.json
run F5 re_4               F5/2_re_4.json
run F5 re_6               F5/3_re_6.json
run F5 re_8               F5/4_re_8.json
barrier

echo "F6  asking people"
run F6_nachfrage p_000    F6_nachfrage/1_p_000.json
run F6_nachfrage p_020    F6_nachfrage/2_p_020.json
run F6_nachfrage p_040    F6_nachfrage/3_p_040.json
run F6_nachfrage p_060    F6_nachfrage/4_p_060.json
run F6_nachfrage p_080    F6_nachfrage/5_p_080.json
run F6_nachfrage p_100    F6_nachfrage/6_p_100.json
run F6_nachfrage gleichverteilt_p_010 F6_nachfrage/7_gleichverteilt_p_010.json
run F6_nachfrage gleichverteilt_p_000 F6_nachfrage/8_gleichverteilt_p_000.json
run F6_nachfrage gleichverteilt_p_020 F6_nachfrage/9_gleichverteilt_p_020.json
run F6_nachfrage gleichverteilt_p_040 F6_nachfrage/10_gleichverteilt_p_040.json
run F6_nachfrage gleichverteilt_p_060 F6_nachfrage/11_gleichverteilt_p_060.json
run F6_nachfrage gleichverteilt_p_080 F6_nachfrage/12_gleichverteilt_p_080.json
barrier

echo "F0_re2  time budget at re = 2 m"
run F0_re2 tb_02min       F0_re2/1_tb_02min.json
run F0_re2 tb_04min       F0_re2/2_tb_04min.json
run F0_re2 tb_06min       F0_re2/3_tb_06min.json
run F0_re2 tb_08min       F0_re2/4_tb_08min.json
run F0_re2 tb_10min       F0_re2/5_tb_10min.json
run F0_re2 tb_12min       F0_re2/6_tb_12min.json
run F0_re2 tb_14min       F0_re2/7_tb_14min.json
run F0_re2 tb_16min       F0_re2/8_tb_16min.json
run F0_re2 tb_18min       F0_re2/9_tb_18min.json
run F0_re2 tb_20min       F0_re2/10_tb_20min.json
barrier

echo "F0_re4  time budget at re = 4 m"
run F0_re4 tb_02min       F0_re4/1_tb_02min.json
run F0_re4 tb_04min       F0_re4/2_tb_04min.json
run F0_re4 tb_06min       F0_re4/3_tb_06min.json
run F0_re4 tb_08min       F0_re4/4_tb_08min.json
run F0_re4 tb_10min       F0_re4/5_tb_10min.json
run F0_re4 tb_12min       F0_re4/6_tb_12min.json
run F0_re4 tb_14min       F0_re4/7_tb_14min.json
run F0_re4 tb_16min       F0_re4/8_tb_16min.json
run F0_re4 tb_18min       F0_re4/9_tb_18min.json
run F0_re4 tb_20min       F0_re4/10_tb_20min.json
barrier

echo "F0_re6  time budget at re = 6 m"
run F0_re6 tb_02min       F0_re6/1_tb_02min.json
run F0_re6 tb_04min       F0_re6/2_tb_04min.json
run F0_re6 tb_06min       F0_re6/3_tb_06min.json
run F0_re6 tb_08min       F0_re6/4_tb_08min.json
run F0_re6 tb_10min       F0_re6/5_tb_10min.json
run F0_re6 tb_12min       F0_re6/6_tb_12min.json
run F0_re6 tb_14min       F0_re6/7_tb_14min.json
run F0_re6 tb_16min       F0_re6/8_tb_16min.json
run F0_re6 tb_18min       F0_re6/9_tb_18min.json
run F0_re6 tb_20min       F0_re6/10_tb_20min.json
barrier

echo "F0_re6_ohne  time budget at re = 6 m without prior knowledge"
run F0_re6_ohne tb_02min  F0_re6_ohne/1_tb_02min.json
run F0_re6_ohne tb_04min  F0_re6_ohne/2_tb_04min.json
run F0_re6_ohne tb_06min  F0_re6_ohne/3_tb_06min.json
run F0_re6_ohne tb_08min  F0_re6_ohne/4_tb_08min.json
run F0_re6_ohne tb_10min  F0_re6_ohne/5_tb_10min.json
run F0_re6_ohne tb_12min  F0_re6_ohne/6_tb_12min.json
run F0_re6_ohne tb_14min  F0_re6_ohne/7_tb_14min.json
run F0_re6_ohne tb_16min  F0_re6_ohne/8_tb_16min.json
run F0_re6_ohne tb_18min  F0_re6_ohne/9_tb_18min.json
run F0_re6_ohne tb_20min  F0_re6_ohne/10_tb_20min.json
barrier

echo "F0_re8  time budget at re = 8 m"
run F0_re8 tb_02min       F0_re8/1_tb_02min.json
run F0_re8 tb_04min       F0_re8/2_tb_04min.json
run F0_re8 tb_06min       F0_re8/3_tb_06min.json
run F0_re8 tb_08min       F0_re8/4_tb_08min.json
run F0_re8 tb_10min       F0_re8/5_tb_10min.json
run F0_re8 tb_12min       F0_re8/6_tb_12min.json
run F0_re8 tb_14min       F0_re8/7_tb_14min.json
run F0_re8 tb_16min       F0_re8/8_tb_16min.json
run F0_re8 tb_18min       F0_re8/9_tb_18min.json
run F0_re8 tb_20min       F0_re8/10_tb_20min.json
barrier

echo "F0_re8_ohne  time budget at re = 8 m without prior knowledge"
run F0_re8_ohne tb_02min  F0_re8_ohne/1_tb_02min.json
run F0_re8_ohne tb_04min  F0_re8_ohne/2_tb_04min.json
run F0_re8_ohne tb_06min  F0_re8_ohne/3_tb_06min.json
run F0_re8_ohne tb_08min  F0_re8_ohne/4_tb_08min.json
run F0_re8_ohne tb_10min  F0_re8_ohne/5_tb_10min.json
run F0_re8_ohne tb_12min  F0_re8_ohne/6_tb_12min.json
run F0_re8_ohne tb_14min  F0_re8_ohne/7_tb_14min.json
run F0_re8_ohne tb_16min  F0_re8_ohne/8_tb_16min.json
run F0_re8_ohne tb_18min  F0_re8_ohne/9_tb_18min.json
run F0_re8_ohne tb_20min  F0_re8_ohne/10_tb_20min.json
barrier

echo "P2  one year instead of 30 days, for the discussion"
run P2_langzeit beta_smoothed    P2_langzeit/1_beta_smoothed.json
run P2_langzeit wahre_verteilung P2_langzeit/2_wahre_verteilung.json
barrier

echo
echo "all runs done, now: python3 tools/paper_csv.py"
