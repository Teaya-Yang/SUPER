#!/usr/bin/env python3
"""Build a self-contained HTML viewer from extract_replans.py output.

Runs anywhere with Python 3 (no ROS needed):
    python3 make_viewer.py run_replans.json run_replans.html --label "benchmark_dense" --bound 0.8

--bound is the planner's corridor_bound_dis for this run, drawn as a reference
line on the deviation chart. Omit it to leave the line out.
"""
import argparse
import json
import os


def rnd(v, nd=2):
    if isinstance(v, float):
        return round(v, nd)
    if isinstance(v, list):
        return [rnd(x, nd) for x in v]
    return v


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("json_in")
    ap.add_argument("html_out")
    ap.add_argument("--label", default=None, help="run name shown above the title")
    ap.add_argument("--bound", type=float, default=None, help="corridor_bound_dis of the run [m]")
    a = ap.parse_args()

    with open(a.json_in) as f:
        d = json.load(f)
    for c in d["cycles"]:
        c.pop("committed", None)  # not drawn; keeps the page small
        for k in ("guide", "exp", "backup", "sfc"):
            c[k] = rnd(c[k])
    d["odom"] = rnd(d["odom"])
    d["meta"] = {"label": a.label or os.path.basename(a.json_in), "bound": a.bound}

    tmpl_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "viewer_template.html")
    with open(tmpl_path) as f:
        tmpl = f.read()
    payload = json.dumps(d, separators=(",", ":")).replace("</", "<\\/")
    with open(a.html_out, "w") as f:
        f.write(tmpl.replace("__DATA__", payload, 1))
    print(f"wrote {a.html_out} ({os.path.getsize(a.html_out) / 1e6:.1f} MB)")


if __name__ == "__main__":
    main()
