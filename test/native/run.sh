#!/bin/bash
# Build and run the native test suite.
#   ./run.sh              build with ASan+UBSan and run
#   ./run.sh --coverage   build instrumented, run, and print a merged gcov report
#
# Two configurations are built. The main suite runs the default two-pack build;
# single/ runs a second binary with -DNUM_PACKS_CFG=1, which is the only way to
# reach the firmware's single-pack handling (notably the dead-cell response,
# which must inhibit outright when there is no second pack to fall back on).
# The coverage report is the union: a line counts as covered if either
# configuration executed it.
set -e
cd "$(dirname "$0")"
ROOT=../..
SRC=$(ls $ROOT/src/*.cpp)
FLAGS="-std=gnu++14 -g -I sim -I $ROOT/include -I $ROOT/src"

build_and_run() {           # $1 = obj dir, $2 = binary, $3 = extra flags, $4.. = sources
    local dir="$1" bin="$2" extra="$3"; shift 3
    rm -rf "$dir" && mkdir -p "$dir"
    local objs="" f o
    for f in "$@"; do
        o="$dir/$(basename ${f%.cpp}).o"
        g++-14 $FLAGS $extra --coverage -O0 -pthread -c "$f" -o "$o"
        objs="$objs $o"
    done
    g++-14 --coverage -pthread -o "$bin" $objs
}

if [ "$1" = "--coverage" ]; then
    rm -f *.gcov
    rc=0

    build_and_run cov testrun-cov "" test_*.cpp sim/sim.cpp sim/unit.cpp $SRC
    ./testrun-cov > /dev/null || rc=$?

    build_and_run cov1 testrun-cov-single "-DNUM_PACKS_CFG=1" \
        single/test_single_main.cpp sim/sim.cpp sim/unit.cpp $SRC
    ./testrun-cov-single > /dev/null || rc=$?

    # Per-configuration .gcov, kept apart so the line hits can be unioned.
    # gcov has to run from here so the source paths recorded at compile time
    # resolve; the output is moved aside between the two passes.
    rm -rf gcov2 gcov1 && mkdir -p gcov2 gcov1
    gcov-14 -o cov  $SRC >/dev/null 2>&1 || true
    mv -f *.gcov gcov2/ 2>/dev/null || true
    gcov-14 -o cov1 $SRC >/dev/null 2>&1 || true
    mv -f *.gcov gcov1/ 2>/dev/null || true

    echo
    echo "=== line coverage (project sources, both pack configurations) ==="
    total=0; covered=0
    for f in $SRC; do
        b=$(basename $f)
        [ -f "gcov2/$b.gcov" ] || continue
        # A line is covered if either build executed it; executable-line sets are
        # identical because both compile the same source, so union by line number.
        read c t <<< $(awk '
            function ln(s){ sub(/^[^:]*:/,"",s); sub(/:.*/,"",s); gsub(/ /,"",s); return s }
            FNR==NR { if ($0 ~ /^ *#####:/) uncov[ln($0)]=1;
                      else if ($0 ~ /^ *[0-9]+[*]?:/) cov[ln($0)]=1; next }
            { if ($0 ~ /^ *[0-9]+[*]?:/) cov[ln($0)]=1 }
            END { t=0; c=0;
                  for (l in cov)   { t++; c++ }
                  for (l in uncov) if (!(l in cov)) t++;
                  print c, t }' "gcov2/$b.gcov" "gcov1/$b.gcov")
        total=$((total+t)); covered=$((covered+c))
        awk -v n="$b" -v c="$c" -v t="$t" 'BEGIN{ if(t) printf "  %-18s %6.1f%%   %4d/%-4d\n", n, 100*c/t, c, t }'
    done
    awk -v c="$covered" -v t="$total" 'BEGIN{ printf "  %-18s %6.1f%%   %4d/%-4d\n", "TOTAL", 100*c/t, c, t }'
    echo
    echo "lines covered by neither configuration:"
    for f in $SRC; do
        b=$(basename $f)
        [ -f "gcov2/$b.gcov" ] || continue
        awk -v n="$b" '
            function ln(s){ sub(/^[^:]*:/,"",s); sub(/:.*/,"",s); gsub(/ /,"",s); return s }
            FNR==NR { if ($0 ~ /^ *[0-9]+[*]?:/) cov[ln($0)]=1; next }
            $0 ~ /^ *#####:/ { l=ln($0); if (!(l in cov)) { t=$0; sub(/^ *#####: *[0-9]*:/,"",t); printf "  %s:%s:%s\n", n, l, t } }
        ' "gcov1/$b.gcov" "gcov2/$b.gcov"
    done
    exit $rc
else
    g++-14 $FLAGS -pthread -fsanitize=address,undefined -o testrun test_*.cpp sim/sim.cpp sim/unit.cpp $SRC
    ./testrun
    echo
    echo "=== single-pack configuration (-DNUM_PACKS_CFG=1) ==="
    g++-14 $FLAGS -DNUM_PACKS_CFG=1 -pthread -fsanitize=address,undefined \
        -o testrun-single single/test_single_main.cpp sim/sim.cpp sim/unit.cpp $SRC
    ./testrun-single
fi
