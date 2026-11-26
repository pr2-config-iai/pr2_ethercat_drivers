#!/usr/bin/env python3
"""Frequency response and arm model from velocity_step_test.py --sine CSVs (decision D30).

For each sine phase of each CSV (the frequency is taken from the reference's zero crossings; the
fade-in and fade-out periods are left out) it prints

  - |V/R| and lag: the measured velocity's fundamental against the reference's,
  - J and b: the arm's impedance, effort fundamental over velocity fundamental, as an apparent
    inertia (imaginary part / w) and damping (real part). A rigid arm has the same J at every
    frequency; a J that rises with frequency means a compliance between motor and arm,
  - the fraction of samples at the clamp (|effort| >= 0.98 --clamp). Phases at the clamp for 5 %
    of the time or more are marked '*' and left out of the fit: their response is the clamp's.

With --fit it fits, over the unclamped phases of all CSVs, a rigid inertia (J s + b) and a two-mass
model seen from the motor, Z(s) = Jm s + bm + (k + c s) Jl s / (Jl s^2 + c s + k), and, for every
CSV given with its gains (CSV:P:I), compares the measured |V/R| with both models' closed loop
C / (Z + C), C = P + I / s. --predict P:I[,P:I...] prints the two-mass closed loop for other gains,
also with feedforward of the rigid fit (J a + b v). The models have no delays.

  velocity_sine_analysis.py --joint l_shoulder_pan_joint run_i800.csv:18:800 run_i400.csv:18:400 \\
      --fit --predict 30:800,18:1600
"""
import argparse
import csv
import math
import sys

import numpy as np

ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument('runs', nargs='+', help='CSV from velocity_step_test.py --sine, optionally CSV:P:I')
ap.add_argument('--joint', required=True, help='the joint the sines were sent to')
ap.add_argument('--clamp', type=float, default=9.0, help="the controller's output clamp in Nm (default 9)")
ap.add_argument('--fit', action='store_true', help='fit the rigid and the two-mass model')
ap.add_argument('--predict', default=None, help='P:I[,P:I...]: closed loop of the two-mass fit for these gains')
args = ap.parse_args()
if args.predict and not args.fit:
    ap.error('--predict needs --fit')


def phases_of(ref, zero_run=50):
    # Runs of nonzero reference; short zero runs are the reference crossing zero (3 decimals).
    phases, cur, zeros = [], [], 0
    for k, r in enumerate(ref):
        if r != 0.0:
            cur += [k] if not zeros else list(range(k - zeros, k + 1))
            zeros = 0
        elif cur:
            zeros += 1
            if zeros > zero_run:
                phases.append(cur)
                cur, zeros = [], 0
    if cur:
        phases.append(cur)
    return phases


def analyse(path):
    rows = list(csv.DictReader(open(path)))
    if not rows or args.joint + '_vel' not in rows[0]:
        sys.exit('%s: no %s columns' % (path, args.joint))
    col = lambda key: np.array([float(r[key]) for r in rows])
    t, ref = col('t'), col('ref')
    v, e = col(args.joint + '_vel'), col(args.joint + '_eff')
    out = []
    for ph in phases_of(ref):
        ph = np.array(ph)
        duration = t[ph[-1]] - t[ph[0]]
        nz = ph[ref[ph] != 0.0]
        cross = t[nz[1:][np.diff(np.sign(ref[nz])) != 0]]
        if duration < 1.0 or len(cross) < 4:
            continue
        f = round((len(cross) - 1) / (2 * (cross[-1] - cross[0])), 2)
        t0, period = t[ph[0]], 1.0 / f
        full = ph[(t[ph] - t0 >= period) & (t[ph] - t0 <= duration - period)]
        if len(full) < 20:
            continue
        w = 2 * math.pi * f
        rot = np.exp(-1j * w * (t[full] - t0))
        R, V, E = (2 * np.mean(s[full] * rot) for s in (ref, v, e))
        out.append(dict(f=f, R=R, V=V, E=E, sat=float(np.mean(np.abs(e[full]) >= 0.98 * args.clamp))))
    return out


def closed_loop(Z, p, i, w, ff=None):
    s = 1j * w
    C = p + i / s
    return (C + (ff(s) if ff else 0.0)) / (Z(s) + C)


runs = []
for spec in args.runs:
    parts = spec.split(':')
    gains = (float(parts[1]), float(parts[2])) if len(parts) == 3 else None
    runs.append((parts[0], gains, analyse(parts[0])))

for path, gains, phases in runs:
    print('%s%s' % (path, ' (p %g, i %g)' % gains if gains else ''))
    for ph in phases:
        Zm = ph['E'] / ph['V']
        T = ph['V'] / ph['R']
        print('  %5.2f Hz: |V/R| %.2f, lag %+4.0f deg | J %.2f kg m^2, b %5.2f Nm s/rad | at clamp %3.0f %%%s' % (
            ph['f'], abs(T), -math.degrees(np.angle(T)), Zm.imag / (2 * math.pi * ph['f']), Zm.real,
            100 * ph['sat'], ' *' if ph['sat'] >= 0.05 else ''))

if not args.fit:
    sys.exit(0)

from scipy.optimize import least_squares  # noqa: E402 (only needed for --fit)

pts = [(2 * math.pi * ph['f'], ph['E'] / ph['V']) for _, _, phases in runs for ph in phases if ph['sat'] < 0.05]
if len(pts) < 3:
    sys.exit('fewer than 3 unclamped phases, nothing to fit')


def rel_residuals(model):
    def res(p):
        d = [(model(p, 1j * w) - z) / abs(z) for w, z in pts]
        return [x for c in d for x in (c.real, c.imag)]
    return res


def rigid(p, s):
    return p[0] * s + p[1]


def two_mass(p, s):
    Jm, bm, Jl, k, c = p
    return Jm * s + bm + (k + c * s) * Jl * s / (Jl * s * s + c * s + k)


rg = least_squares(rel_residuals(rigid), [3.0, 3.0], bounds=([0, 0], [100, 100]))
best = None
for k0 in (200, 500, 1000, 3000):
    for jm0 in (0.5, 1.5, 2.5):
        sol = least_squares(rel_residuals(two_mass), [jm0, 3, 1.0, k0, 5],
                            bounds=([0, 0, 0, 1, 0], [50, 100, 50, 1e5, 1000]))
        if best is None or sol.cost < best.cost:
            best = sol
rms = lambda sol: math.sqrt(2 * sol.cost / len(pts))
Jm, bm, Jl, k, c = best.x
print('\nfit over %d unclamped phases (relative rms error):' % len(pts))
print('  rigid:    J %.2f kg m^2, b %.2f Nm s/rad (%.3f)' % (rg.x[0], rg.x[1], rms(rg)))
print('  two-mass: Jm %.2f, bm %.2f, Jl %.2f kg m^2, k %.0f Nm/rad, c %.2f Nm s/rad (%.3f); '
      'resonance with the motor held %.2f Hz' % (Jm, bm, Jl, k, c, rms(best), math.sqrt(k / Jl) / (2 * math.pi)))
Z2 = lambda s: two_mass(best.x, s)
Zr = lambda s: rigid(rg.x, s)
for path, gains, phases in runs:
    if not gains:
        continue
    print('\n%s (p %g, i %g): |V/R| measured / two-mass / rigid' % ((path,) + gains))
    for ph in phases:
        w = 2 * math.pi * ph['f']
        print('  %5.2f Hz: %.2f / %.2f / %.2f%s' % (
            ph['f'], abs(ph['V'] / ph['R']), abs(closed_loop(Z2, *gains, w)), abs(closed_loop(Zr, *gains, w)),
            '  (measured at the clamp)' if ph['sat'] >= 0.05 else ''))
if args.predict:
    fs = np.linspace(0.2, 4.0, 381)
    ff = lambda s: rg.x[0] * s + rg.x[1]
    print('\ntwo-mass closed loop |V/R| (peak, and at 0.5 / 1 / 1.5 / 2 / 3 Hz):')
    for spec in args.predict.split(','):
        p, i = (float(x) for x in spec.split(':'))
        for name, f_ff in (('', None), (' + feedforward', ff)):
            mag = np.array([abs(closed_loop(Z2, p, i, 2 * math.pi * f, f_ff)) for f in fs])
            at = '  '.join('%.2f' % mag[np.argmin(abs(fs - f))] for f in (0.5, 1, 1.5, 2, 3))
            print('  p %g, i %g%s: peak %.2f at %.2f Hz | %s' % (p, i, name, mag.max(), fs[mag.argmax()], at))
