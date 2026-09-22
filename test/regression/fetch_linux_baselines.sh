#!/usr/bin/env bash
#
# fetch_linux_baselines.sh — regenerate test/regression/baselines/linux-x86_64/
#
# The Linux baselines cannot be generated on macOS (they differ by a few ULP —
# see run_regression.py's platform_tag()), so they are produced by a CI job and
# downloaded here. Run this after regenerating the macOS set, and commit both
# together.
#
# Usage:
#     test/regression/fetch_linux_baselines.sh [git-ref]
#
# `git-ref` is the branch the workflow runs on; it defaults to the current
# branch. The branch must already be pushed — GitHub can only run a workflow
# from a ref it has.

set -euo pipefail

WORKFLOW="regression_linux_baselines.yml"
ARTIFACT="linux-x86_64-baselines"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEST="$HERE/baselines/linux-x86_64"

REF="${1:-$(git -C "$HERE" rev-parse --abbrev-ref HEAD)}"

if ! command -v gh >/dev/null 2>&1; then
    echo "ERROR: the GitHub CLI (gh) is not installed." >&2
    echo "       https://cli.github.com — 'brew install gh' on macOS." >&2
    exit 1
fi

if ! gh auth status >/dev/null 2>&1; then
    echo "ERROR: gh is not authenticated. Run 'gh auth login'." >&2
    exit 1
fi

latest_run_id() {
    gh run list --workflow "$WORKFLOW" --branch "$REF" \
        --limit 1 --json databaseId --jq '.[0].databaseId' 2>/dev/null || true
}

# Remember the newest run before dispatching. `gh workflow run` prints no run
# id, so the new run has to be recognised by the id changing — without this,
# a previous run of the same workflow would be watched and downloaded instead.
BEFORE="$(latest_run_id)"

echo "Dispatching $WORKFLOW on ref '$REF'..."
gh workflow run "$WORKFLOW" --ref "$REF"

echo "Waiting for the run to appear..."
RUN_ID=""
for _ in $(seq 1 30); do
    sleep 2
    CANDIDATE="$(latest_run_id)"
    if [ -n "$CANDIDATE" ] && [ "$CANDIDATE" != "$BEFORE" ]; then
        RUN_ID="$CANDIDATE"
        break
    fi
done

if [ -z "$RUN_ID" ]; then
    echo "ERROR: no new run appeared for $WORKFLOW on '$REF'." >&2
    echo "       Check that the branch is pushed and the workflow exists on it:" >&2
    echo "       gh run list --workflow $WORKFLOW --branch $REF" >&2
    exit 1
fi

echo "Run $RUN_ID: $(gh run view "$RUN_ID" --json url --jq .url)"
echo "Watching (this takes a few minutes)..."
if ! gh run watch "$RUN_ID" --exit-status; then
    echo "ERROR: the workflow run failed. See the log:" >&2
    echo "       gh run view $RUN_ID --log-failed" >&2
    exit 1
fi

echo "Downloading $ARTIFACT into $DEST ..."
mkdir -p "$DEST"
rm -f "$DEST"/scenario_*.npz "$DEST"/PROVENANCE.json
gh run download "$RUN_ID" --name "$ARTIFACT" --dir "$DEST"

echo
echo "Done. Files in baselines/linux-x86_64/:"
ls "$DEST"
echo
echo "Review 'git status', then commit them alongside the macOS set."
