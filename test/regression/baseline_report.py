#!/usr/bin/env python3
"""
baseline_report.py — a reviewable report for a deliberate baseline change.

`compare_baselines.py` prints the evidence; this builds a self-contained HTML
page out of it — per-array magnitudes, first-divergence times, and the
before/after/difference plot for every scenario that moved — so the change can
be reviewed by someone who was not at the keyboard when it happened.

    python test/regression/baseline_report.py --out /tmp/report
    python test/regression/baseline_report.py --out /tmp/report \
        --against HEAD~1 --narrative note.html --notes notes.json

Writes `<out>/index.html` and `<out>/plots/`, then prints the file map for
publishing the directory as an artifact.

What it cannot write for you is *why*. The numbers are evidence only against a
prediction you made first — see "Changing a baseline on purpose" in README.md.
Pass that argument in with `--narrative`: an HTML fragment (`<section
class="prose">` blocks) dropped in above the scenario list. Without one the page
says, in place of the argument, that nobody made it.

`--notes` takes JSON mapping a scenario number to `{"tag": ..., "why": ...}`,
for labelling scenarios whose cause differs from the rest:

    {"2":  {"tag": "inputs + DT", "why": "vane and heading now reach the controller"},
     "13": {"tag": "unchanged", "why": "no filter in the loop — torque is pinned"}}

A scenario named in `--notes` that did not move is listed without a plot, which
is how you account for the ones that *should* have held still.
"""

import argparse
import html
import json
import os
import shutil
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from compare_baselines import (  # noqa: E402
    PLATFORM_TAG, changed_scenarios, compare, plot,
)
from scenarios import SCENARIOS  # noqa: E402

STYLE = """<style>
:root {
  --ground:#f6f7f9; --panel:#fff; --ink:#16202a; --ink-2:#53616e; --ink-3:#7b8896;
  --rule:#dfe4ea; --rule-2:#eef1f5; --accent:#14657f; --accent-soft:#e3eef2;
  --moved:#9a5410; --moved-soft:#f6ebdd; --still:#2f6b47; --still-soft:#e3efe7;
  --sans:"IBM Plex Sans",ui-sans-serif,system-ui,sans-serif;
  --serif:"Source Serif 4",Georgia,"Times New Roman",serif;
  --mono:"IBM Plex Mono",ui-monospace,"SF Mono",Menlo,monospace;
}
@media (prefers-color-scheme:dark){:root:not([data-theme="light"]){
  --ground:#10161c; --panel:#161e26; --ink:#e4eaf0; --ink-2:#a3b1bf; --ink-3:#7c8a98;
  --rule:#26313c; --rule-2:#1d262f; --accent:#63b6d2; --accent-soft:#17303b;
  --moved:#d99a55; --moved-soft:#33261a; --still:#79bb93; --still-soft:#1a2c22;}}
:root[data-theme="dark"]{
  --ground:#10161c; --panel:#161e26; --ink:#e4eaf0; --ink-2:#a3b1bf; --ink-3:#7c8a98;
  --rule:#26313c; --rule-2:#1d262f; --accent:#63b6d2; --accent-soft:#17303b;
  --moved:#d99a55; --moved-soft:#33261a; --still:#79bb93; --still-soft:#1a2c22;}
body{background:var(--ground);color:var(--ink);font-family:var(--serif);
  font-size:16.5px;line-height:1.62;-webkit-font-smoothing:antialiased}
.wrap{max-width:940px;margin:0 auto;padding-inline:20px;padding-block:48px 72px}
p,li{max-width:68ch}
h1,h2,h3,.eyebrow,.tag,th,.scn-n,.fig{font-family:var(--sans);text-wrap:balance}
h1{font-size:clamp(1.9rem,1.3rem + 2.4vw,2.85rem);font-weight:600;
  letter-spacing:-.02em;line-height:1.12;margin:0 0 14px}
h2{font-size:1.32rem;font-weight:600;letter-spacing:-.01em;margin:0 0 4px}
h3{font-size:1.02rem;font-weight:600;margin:0;letter-spacing:-.005em}
.eyebrow{font-size:.72rem;font-weight:500;letter-spacing:.13em;text-transform:uppercase;
  color:var(--accent);margin:0 0 18px}
.lede{font-size:1.12rem;color:var(--ink-2);margin:0 0 26px}
code,.mono{font-family:var(--mono);font-size:.87em}
code{background:var(--rule-2);padding:.1em .34em;border-radius:3px;color:var(--ink)}
a{color:var(--accent)}
.mast{border-bottom:2px solid var(--ink);padding-bottom:28px}
.meta{display:flex;flex-wrap:wrap;gap:8px 26px;font-family:var(--mono);
  font-size:.76rem;color:var(--ink-3);margin-top:22px}
.meta b{color:var(--ink-2);font-weight:500}
.figs{display:grid;grid-template-columns:repeat(auto-fit,minmax(150px,1fr));gap:1px;
  background:var(--rule);border:1px solid var(--rule);margin:34px 0 44px}
.fig{background:var(--panel);padding:16px 18px}
.fig .v{font-family:var(--sans);font-size:1.72rem;font-weight:600;letter-spacing:-.03em;
  font-variant-numeric:tabular-nums;line-height:1.1}
.fig .l{font-size:.73rem;color:var(--ink-3);letter-spacing:.04em;margin-top:5px;
  font-family:var(--sans)}
section.prose{margin-block:38px}
section.prose h2{margin-bottom:10px}
.sub{font-family:var(--mono);font-size:.72rem;letter-spacing:.1em;text-transform:uppercase;
  color:var(--ink-3);margin:0 0 14px}
.callout{border-left:3px solid var(--accent);background:var(--accent-soft);
  padding:16px 20px;margin:24px 0}
.callout p{margin:0;max-width:62ch}
.callout p + p{margin-top:10px}
.tw{overflow-x:auto;margin:18px 0 8px}
table{border-collapse:collapse;width:100%;font-size:.85rem}
th{text-align:left;font-weight:500;font-size:.71rem;letter-spacing:.08em;
  text-transform:uppercase;color:var(--ink-3);border-bottom:1px solid var(--rule);
  padding:0 14px 7px 0;white-space:nowrap}
td{padding:7px 14px 7px 0;border-bottom:1px solid var(--rule-2);font-family:var(--mono);
  font-size:.8rem;color:var(--ink-2);white-space:nowrap}
td.k{color:var(--ink)}
.n{text-align:right;font-variant-numeric:tabular-nums;padding-right:0}
th.n{padding-right:0}
table.wide td:first-child,table.wide th:first-child{white-space:normal}
table.wide td:last-child{white-space:normal;font-family:var(--serif);font-size:.92rem}
.scn{border-top:1px solid var(--rule);padding-block:26px 34px}
.scn-h{display:flex;align-items:center;gap:12px;flex-wrap:wrap}
.scn-n{font-family:var(--mono);font-size:.8rem;font-weight:500;color:var(--ink-3);
  border:1px solid var(--rule);border-radius:2px;padding:1px 7px;
  font-variant-numeric:tabular-nums}
.scn-h h3{flex:1 1 auto;min-width:200px}
.tag{font-size:.7rem;font-weight:500;letter-spacing:.05em;padding:2px 9px;
  border-radius:2px;white-space:nowrap}
.tag-d{background:var(--rule-2);color:var(--ink-2)}
.tag-i{background:var(--moved-soft);color:var(--moved)}
.tag-s{background:var(--still-soft);color:var(--still)}
.why{color:var(--ink-2);font-size:.95rem;margin:8px 0 0}
.scn-still .why{margin-bottom:4px}
figure{margin:18px 0 0}
figure img{display:block;width:100%;height:auto;background:#fff;
  border:1px solid var(--rule);border-radius:2px}
figcaption{font-family:var(--sans);font-size:.75rem;color:var(--ink-3);margin-top:8px}
footer{border-top:2px solid var(--ink);margin-top:44px;padding-top:20px;
  font-size:.9rem;color:var(--ink-2)}
ol.steps{padding-left:1.15em}
ol.steps li{margin-bottom:9px}
@media (max-width:520px){body{font-size:16px}.wrap{padding-block:32px 52px}}
</style>"""

FONTS = ('<link rel="stylesheet" href="https://fonts.googleapis.com/css2?'
         'family=IBM+Plex+Mono:wght@400;500&family=IBM+Plex+Sans:wght@400;500;600'
         '&family=Source+Serif+4:opsz,wght@8..60,400;8..60,600&display=swap">')

NO_NARRATIVE = """<section class="prose">
  <h2>Why these moved</h2>
  <div class="callout"><p>No argument was supplied with this report. The numbers
  below are evidence only against a prediction made <i>before</i> the change; on
  their own they are a diff. Re-run with <code>--narrative</code> and say what
  changed, what you expected to move, and why every row below is explainable.</p>
  </div>
</section>"""


def _fmt_rows(rows):
    trs = []
    for key, max_abs, frac, n_diff, first_t, note in rows:
        if note:
            trs.append(f'<tr><td class="k">{html.escape(key)}</td>'
                       f'<td colspan="4">{html.escape(note)}</td></tr>')
            continue
        rel = "was all zero" if frac != frac else f"{frac:.3%} of peak"
        trs.append(f'<tr><td class="k">{html.escape(key)}</td>'
                   f'<td class="n">{max_abs:.6g}</td><td class="n">{rel}</td>'
                   f'<td class="n">{n_diff:,}</td><td class="n">{first_t:.3f} s</td></tr>')
    return "".join(trs)


def _title(num):
    s = SCENARIOS.get(num)
    return s.title if s else f"scenario {num}"


def section(num, rows, note, has_plot):
    tag = note.get("tag", "moved")
    why = note.get("why", "")
    cls = "tag-i" if note.get("tag") else "tag-d"
    if not has_plot:
        cls = "tag-s"
    body = [f'<section class="scn{"" if has_plot else " scn-still"}" id="s{num}">',
            '  <header class="scn-h">',
            f'    <span class="scn-n">{num}</span>',
            f'    <h3>{html.escape(_title(num))}</h3>',
            f'    <span class="tag {cls}">{html.escape(tag)}</span>',
            '  </header>']
    if why:
        body.append(f'  <p class="why">{html.escape(why)}</p>')
    if rows:
        body += ['  <div class="tw"><table>',
                 '    <thead><tr><th>array</th><th class="n">max |&Delta;|</th>'
                 '<th class="n">vs peak</th><th class="n">samples</th>'
                 '<th class="n">first &Delta;</th></tr></thead>',
                 f'    <tbody>{_fmt_rows(rows)}</tbody>',
                 '  </table></div>']
    if has_plot:
        body += [f'  <figure><img src="plots/scenario_{num}.png" loading="lazy"',
                 f'    alt="Scenario {num}: before, after and difference traces">',
                 '    <figcaption>Old baseline (blue) against new (orange), with the '
                 'difference at right.</figcaption>',
                 '  </figure>']
    body.append('</section>')
    return "\n".join(body)


def main():
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--out", required=True, help="Directory to write the report into.")
    p.add_argument("--against", default="HEAD", help="Git ref to compare with (default HEAD).")
    p.add_argument("--narrative", help="HTML fragment: the argument for the change.")
    p.add_argument("--notes", help="JSON: {scenario: {tag, why}} for per-scenario labels.")
    p.add_argument("--title", default="Baseline Change", help="Page title.")
    p.add_argument("--lede", default="", help="One-sentence standfirst under the title.")
    args = p.parse_args()

    notes = {}
    if args.notes:
        with open(args.notes) as fh:
            notes = {int(k): v for k, v in json.load(fh).items()}

    moved = changed_scenarios(args.against)
    if not moved:
        print(f"No baseline differs from {args.against} — nothing to report.")
        return 1

    plots_dir = os.path.join(args.out, "plots")
    os.makedirs(plots_dir, exist_ok=True)

    sections, file_map = [], {}
    for num in moved:
        _, rows = compare(num, args.against)
        src = plot(num, args.against, plots_dir)
        if src:
            dst = os.path.join(plots_dir, f"scenario_{num}.png")
            if os.path.abspath(src) != os.path.abspath(dst):
                shutil.move(src, dst)
            file_map[f"plots/scenario_{num}.png"] = f"plots/scenario_{num}.png"
        sections.append(section(num, rows, notes.get(num, {}), bool(src)))

    # Scenarios called out in --notes that did NOT move: the ones held still on
    # purpose. They belong in the report exactly because they have no plot.
    for num in sorted(set(notes) - set(moved)):
        sections.append(section(num, [], notes[num], False))

    narrative = NO_NARRATIVE
    if args.narrative:
        with open(args.narrative) as fh:
            narrative = fh.read()

    sha = os.popen(f"git -C {os.path.dirname(HERE)} rev-parse --short {args.against}").read().strip()
    page = f"""<title>{html.escape(args.title)}</title>
{FONTS}
{STYLE}
<div class="wrap">
<header class="mast">
  <p class="eyebrow">ROSCO-C &middot; regression suite &middot; baseline change</p>
  <h1>{html.escape(args.title)}</h1>
  {f'<p class="lede">{html.escape(args.lede)}</p>' if args.lede else ''}
  <div class="meta">
    <span><b>against</b> {html.escape(args.against)} {sha}</span>
    <span><b>platform</b> {PLATFORM_TAG}</span>
    <span><b>moved</b> {len(moved)} of {len(SCENARIOS)} scenarios</span>
  </div>
</header>
{narrative}
<section class="prose">
  <h2>Scenario by scenario</h2>
  <p class="sub">{len(moved)} moved &middot; {len(set(notes) - set(moved))} accounted for without a plot</p>
</section>
{chr(10).join(sections)}
<footer>
  <p>Generated by <code>test/regression/baseline_report.py</code>. Percentages are against
  the <i>old</i> signal's peak, so a channel that used to sit near zero can exceed 100%.
  The procedure these numbers answer to is under &ldquo;Changing a baseline on purpose&rdquo;
  in <code>test/regression/README.md</code>.</p>
</footer>
</div>
"""
    index = os.path.join(args.out, "index.html")
    with open(index, "w") as fh:
        fh.write(page)

    print(f"Wrote {index} ({len(moved)} scenarios, {len(file_map)} plots)")
    print("\nPublish with the Artifact tool:")
    print(f"  file_path: {index}")
    print(f"  root:      {os.path.abspath(args.out)}")
    print(f"  files:     {json.dumps(file_map)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
