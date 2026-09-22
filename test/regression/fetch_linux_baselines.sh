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
#
# The repository is taken from the branch's push remote (origin if it has
# none), so a clone with several remotes — a fork plus its upstream — needs no
# `gh repo set-default`, and the job can never be dispatched against upstream
# by accident.

set -euo pipefail

WORKFLOW="regression_linux_baselines.yml"
ARTIFACT="linux-x86_64-baselines"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEST="$HERE/baselines/linux-x86_64"

# `--run <id>` attaches to a run that is already going instead of dispatching a
# new one — what you want after the script died mid-wait, or when the run was
# started from the Actions tab.
ATTACH_RUN=""
if [ "${1:-}" = "--run" ]; then
    ATTACH_RUN="${2:-}"
    if [ -z "$ATTACH_RUN" ]; then
        echo "ERROR: --run needs a run id." >&2
        exit 1
    fi
    shift 2
fi

if [ "$#" -gt 1 ]; then
    # Catches the hint lines in this script's own errors being pasted back whole.
    echo "ERROR: expected at most one argument (a git ref), got: $*" >&2
    echo "       Usage: fetch_linux_baselines.sh [git-ref]" >&2
    echo "              fetch_linux_baselines.sh --run <run-id>" >&2
    exit 1
fi

REF="${1:-$(git -C "$HERE" rev-parse --abbrev-ref HEAD)}"

REMOTE="$(git -C "$HERE" config "branch.$REF.remote" || echo origin)"
REMOTE_URL="$(git -C "$HERE" remote get-url "$REMOTE")"
# Both URL spellings: https://github.com/OWNER/REPO(.git) and git@github.com:OWNER/REPO(.git)
REPO="$(printf '%s' "$REMOTE_URL" | sed -E 's#^.*github\.com[:/]##; s#\.git$##')"
if [ -z "$REPO" ]; then
    echo "ERROR: could not work out the GitHub repo from remote '$REMOTE' ($REMOTE_URL)." >&2
    exit 1
fi

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
    gh run list --repo "$REPO" --workflow "$WORKFLOW" --branch "$REF" \
        --limit 1 --json databaseId --jq '.[0].databaseId' 2>/dev/null || true
}

if [ -n "$ATTACH_RUN" ]; then
    RUN_ID="$ATTACH_RUN"
    echo "Attaching to existing run $RUN_ID on $REPO (not dispatching)."
else
    # Remember the newest run before dispatching. `gh workflow run` prints no run
    # id, so the new run has to be recognised by the id changing — without this,
    # a previous run of the same workflow would be watched and downloaded instead.
    BEFORE="$(latest_run_id)"

    echo "Dispatching $WORKFLOW on $REPO, ref '$REF'..."
    gh workflow run --repo "$REPO" "$WORKFLOW" --ref "$REF"

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
        echo "       gh run list --repo $REPO --workflow $WORKFLOW --branch $REF" >&2
        exit 1
    fi
fi

echo "Run $RUN_ID: $(gh run view --repo "$REPO" "$RUN_ID" --json url --jq .url)"
echo "Waiting for the run to finish (this takes several minutes)..."

# Poll rather than `gh run watch --exit-status`: watch exits non-zero both when
# the run fails and when a single API call hiccups (HTTP 502s are routine), and
# the two must not be confused — a transient error once reported a perfectly
# healthy run as a failure. Only the run's own conclusion is trusted, and a
# query that fails is retried instead of being believed.
CONCLUSION=""
for _ in $(seq 1 240); do          # 240 x 15s = 60 minutes
    STATE="$(gh run view --repo "$REPO" "$RUN_ID" \
        --json status,conclusion --jq '.status+" "+(.conclusion//"")' 2>/dev/null || true)"
    if [ -z "$STATE" ]; then
        sleep 15                   # transient API error — say nothing, try again
        continue
    fi
    if [ "${STATE%% *}" = "completed" ]; then
        CONCLUSION="${STATE#* }"
        break
    fi
    sleep 15
done

if [ -z "$CONCLUSION" ]; then
    echo "ERROR: the run had not finished after 60 minutes. Check it directly:" >&2
    echo "       gh run view --repo $REPO $RUN_ID" >&2
    exit 1
fi

if [ "$CONCLUSION" != "success" ]; then
    echo "ERROR: the workflow run finished with '$CONCLUSION'. See the log:" >&2
    echo "       gh run view --repo $REPO $RUN_ID --log-failed" >&2
    exit 1
fi

echo "Downloading $ARTIFACT into $DEST ..."
mkdir -p "$DEST"
rm -f "$DEST"/scenario_*.npz "$DEST"/PROVENANCE.json
gh run download --repo "$REPO" "$RUN_ID" --name "$ARTIFACT" --dir "$DEST"

echo
echo "Done. Files in baselines/linux-x86_64/:"
ls "$DEST"
echo
echo "Review 'git status', then commit them alongside the macOS set."
