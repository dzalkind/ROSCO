---
name: baseline-report
description: Produce a reviewable report for a deliberate change to the ROSCO regression baselines — per-array magnitudes, first-divergence times and before/after plots, published as an artifact. Use when a baseline has moved on purpose and someone other than you has to accept it.
---

# Baseline change report

A baseline diff is tens of megabytes of binary; `git diff` says only "differs".
This produces the evidence a reviewer needs to accept or reject the change, as
one page they can open.

**Do not run this to explain away a failure.** If a baseline moved and you did
not intend it to, that is a bug in your change. Go fix it. This skill is for a
move that was decided on deliberately.

## The rule this exists to serve

You predict the change before you look at it, and the evidence has to match your
prediction. A prediction made after seeing the diff is not evidence. The full
procedure is in `test/regression/README.md` under "Regenerating a baseline" —
follow it; this skill covers only the reporting step.

## Steps

1. **Confirm the prediction was written down first.** If the user has not stated
   what they expected to move, ask for it before generating anything. A report
   whose argument is assembled after the fact is worth very little, and the page
   will say so if you leave the narrative out.

2. **Regenerate the affected baselines and leave them in the working tree.** The
   report compares the working tree against a git ref (`HEAD` by default), so
   generate it *before* committing, or pass `--against HEAD~1` afterwards.

3. **Write the notes file** — one entry per scenario whose cause differs from
   the rest, plus every scenario that was expected to move and did not:

   ```json
   {"2":  {"tag": "inputs + DT", "why": "vane and heading now reach the controller"},
    "13": {"tag": "unchanged", "why": "no filter in the loop — torque is pinned at 1.0"}}
   ```

   A scenario named here that did *not* move is listed without a plot. Those
   entries carry real weight: they are how the report accounts for what held
   still, which is half of any prediction.

4. **Write the narrative** as an HTML fragment of `<section class="prose">`
   blocks — the argument for the change, in the author's voice. This is the part
   that cannot be generated. It should cover:
   - what was wrong, concretely, with file and symbol references
   - what was predicted to move, and whether that held
   - the independent checks that closed it — a clean reproducing run, the
     scenarios that correctly held still, and any cross-check against a
     reference implementation
   - anything the numbers would mislead a reader about (see below)

   Available classes: `.callout` for a boxed aside, `.sub` for an uppercase
   kicker, `ol.steps` for a numbered list, `table.wide` for a prose table,
   `.mono` for inline fixed-width.

5. **Generate:**

   ```bash
   python test/regression/baseline_report.py --out /tmp/report \
       --title "<two to four words>" --lede "<one sentence>" \
       --narrative narrative.html --notes notes.json
   ```

   It prints the `file_path`, `root` and `files` map to publish with.

6. **Publish with the Artifact tool**, passing exactly those three values plus
   `icon: "chart"` and a one-sentence `description`. Give the user the link and
   say it is private until they share it.

## Reading the numbers honestly

The report's percentages are against the **old** signal's peak, so:

- A channel that used to sit near zero can report thousands of percent. That is
  a flat trace waking up, not a catastrophe — say so.
- A rate- or saturation-limited channel can report ~100% from a small cause,
  because what moved is *when* it hits the limit, not by how much.
- **First divergence time** is the more diagnostic column. A filter-initialisation
  change shows up in the first few steps; a mode that only engages above rated
  shows up when the wind crosses it. If the time does not match the mechanism you
  are claiming, you have not found the cause yet.

Say these things in the narrative rather than letting the reader draw the wrong
conclusion from a big number.

## After publishing

Name what still has to happen. The usual one: a local `--update-baseline` only
rewrites this machine's platform folder, so the other platform's set is now
stale and CI will stay red until it is regenerated from the same commit. See
"Regenerating the linux-x86_64 baselines" in the README.
