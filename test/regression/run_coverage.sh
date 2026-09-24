#!/usr/bin/env bash
#
# run_coverage.sh — which controller lines and branches do 30 scenarios reach?
#
# Builds libdiscon with coverage instrumentation, runs the full regression
# suite against that build, and reports with gcovr. Local only; there is no CI
# job for this, and there is deliberately no threshold to fail against — the
# number is for reading, not for gating.
#
#   test/regression/run_coverage.sh                  # text summary + HTML
#   test/regression/run_coverage.sh --open           # ...and open the HTML
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
[[ "${1:-}" == "--open" ]] && open_html=1

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

echo "==> Running the regression suite against the instrumented build"
python "$HERE/run_regression.py"

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
echo "Text:  $OUT/coverage.txt"
echo "HTML:  $OUT/index.html"
[[ $open_html -eq 1 ]] && { command -v open >/dev/null && open "$OUT/index.html"; }
echo
echo "Read the low rows as questions, not as failures. A file at 0% is either"
echo "dead code, or a mode no scenario configures — mode_coverage.py --gaps"
echo "tells you which."
