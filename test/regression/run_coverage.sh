#!/usr/bin/env bash
#
# run_coverage.sh — which controller lines and branches do 30 scenarios reach?
#
# Builds libdiscon with coverage instrumentation, runs the full regression
# suite against that build, and reports with gcovr. Local only; there is no CI
# job for this, and there is deliberately no threshold to fail against — the
# number is for reading, not for gating.
#
#   test/regression/run_coverage.sh                  # regression suite only
#   test/regression/run_coverage.sh --all            # ...plus toolbox tests + Examples
#   test/regression/run_coverage.sh --open           # ...and open the HTML
#
# Counters accumulate across every process that loads the instrumented library,
# so --all is the honest picture of what the repo as a whole exercises. It
# reaches things the 30 scenarios structurally cannot: rosco/test/test_checkpoint.py
# is the only thing anywhere that drives the warm-restart path (iStatus == -9),
# and Examples 17a/17b/17c and 33 are the only things that drive ZeroMQ.
# Those extra workloads are allowed to fail without killing the report — they
# need OpenFAST, a ZeroMQ server and a working network, and an environment
# without them is not a coverage finding. The regression suite is not allowed
# to fail.
#
# The report lands in coverage-report/ at the repo root (gitignored). Override
# with COVERAGE_OUT=/some/dir.
#
# Needs gcovr (`pip install gcovr`). On macOS it drives Apple clang's
# `llvm-cov gcov`; on Linux, plain gcov.
#
# The suite must still report ALL IDENTICAL under instrumentation. That is not
# incidental: the build is -O0 where the baselines were captured at
# RelWithDebInfo, so a green run here is a live demonstration that
# -ffp-contract=off does what CMakeLists.txt claims. A mismatch is a finding
# about the flags, not a reason to update a baseline.
#
# Never capture a baseline from this build.

set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
CTRL="$REPO/rosco/controller"
BUILD="$CTRL/build-coverage"
OUT="${COVERAGE_OUT:-$REPO/coverage-report}"
LIB_DIR="$REPO/rosco/lib"

open_html=0
run_all=0
for arg in "$@"; do
  case "$arg" in
    --open) open_html=1 ;;
    --all)  run_all=1 ;;
    *) echo "usage: run_coverage.sh [--all] [--open]" >&2; exit 2 ;;
  esac
done

case "$(uname -s)" in
  Darwin) LIB=libdiscon.dylib; GCOV="llvm-cov gcov" ;;
  *)      LIB=libdiscon.so;    GCOV="gcov" ;;
esac

command -v gcovr >/dev/null || { echo "gcovr not found — pip install gcovr" >&2; exit 1; }

# The suite loads whatever is in rosco/lib/, so the instrumented library has to
# go there. Put the real one back on any exit, including a failed run.
BACKUP="$(mktemp -d)/$LIB"
restore() {
  if [[ -f "$BACKUP" ]]; then
    mv -f "$BACKUP" "$LIB_DIR/$LIB"
    echo "Restored the uninstrumented $LIB"
  fi
}
trap restore EXIT

echo "==> Configuring $BUILD"
cmake -S "$CTRL" -B "$BUILD" -DROSCO_COVERAGE=ON -DCMAKE_BUILD_TYPE=Debug >/dev/null

echo "==> Building"
cmake --build "$BUILD" --parallel >/dev/null

# Counts accumulate across runs, so clear them or the report describes this run
# plus every earlier one.
find "$BUILD" -name '*.gcda' -delete

cp "$LIB_DIR/$LIB" "$BACKUP"
cp "$BUILD/$LIB" "$LIB_DIR/$LIB"

WORKLOADS=()

echo "==> Running the regression suite against the instrumented build"
python "$HERE/run_regression.py"
WORKLOADS+=("regression suite (30 scenarios, ALL IDENTICAL)")

if [[ $run_all -eq 1 ]]; then
  # Everything below is best-effort: these need OpenFAST, a ZeroMQ server and a
  # network. A failure here narrows the report, it is not a finding.
  echo
  echo "==> Toolbox tests (test_checkpoint.py is the only warm-restart driver)"
  if python -m pytest "$REPO/rosco/test" -q --ignore="$REPO/rosco/test/test_examples.py"; then
    WORKLOADS+=("rosco/test")
  else
    WORKLOADS+=("rosco/test — FAILED or partial, coverage below is narrower")
  fi

  echo
  echo "==> Examples (16_external_dll and 17a/b/c + 33 reach code nothing else does)"
  if python -m pytest "$REPO/rosco/test/test_examples.py" -q; then
    WORKLOADS+=("Examples")
  else
    WORKLOADS+=("Examples — FAILED or partial, coverage below is narrower")
  fi
fi

echo "==> Reporting"
mkdir -p "$OUT"
gcovr "$BUILD" \
  --root "$CTRL" \
  --filter "$CTRL/src/" \
  --gcov-executable "$GCOV" \
  --exclude-unreachable-branches \
  --exclude-throw-branches \
  --sort uncovered-percent \
  --txt "$OUT/coverage.txt" \
  --html-details "$OUT/index.html" \
  --json-summary "$OUT/summary.json" \
  --print-summary

echo
echo "Contributing workloads:"
for w in "${WORKLOADS[@]}"; do echo "  - $w"; done
echo
echo "Text:  $OUT/coverage.txt"
echo "HTML:  $OUT/index.html"
[[ $open_html -eq 1 ]] && { command -v open >/dev/null && open "$OUT/index.html"; }
echo
echo "Read the low rows as questions, not as failures. A file at 0% is either"
echo "dead code, or a mode no scenario configures — mode_coverage.py --gaps"
echo "tells you which."
