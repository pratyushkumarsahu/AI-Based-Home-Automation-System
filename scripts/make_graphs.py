#!/usr/bin/env python3
"""Makes graphs + a summary from the robot's recorded run folders.
Usage:
  python3 scripts/make_graphs.py                 # newest runs that contain data
  python3 scripts/make_graphs.py results/RUN ... # specific run folders
  python3 scripts/make_graphs.py --obstacle 0.35 # obstacle stop distance (m)"""
import csv, glob, math, os, sys
import matplotlib
matplotlib.use('Agg')                      # works without a display
import matplotlib.pyplot as plt

ROOT = os.environ.get('RESULTS_DIR', os.path.expanduser('~/home_monitor_ws/results'))
KNOWN = ['Pratyush', 'Arpita', 'Rohit', 'OUTSIDER']      # ignores YOLO junk like "vase"
OBSTACLE = 0.35

args = sys.argv[1:]
if '--obstacle' in args:
    i = args.index('--obstacle'); OBSTACLE = float(args[i + 1]); del args[i:i + 2]


def read(run, name):
    p = os.path.join(run, name)
    if not os.path.exists(p):
        return []
    with open(p, newline='') as f:
        lines = f.read().replace('\r', '').splitlines()
    return [{(k or '').strip(): (v or '').strip() for k, v in r.items()}
            for r in csv.DictReader(lines)]


def col(rows, key):
    out = []
    for r in rows:
        try:
            out.append(float(r[key]))
        except (KeyError, ValueError):
            out.append(float('nan'))
    return out


def has_data(run):
    return len(read(run, 'odom.csv')) > 200 and len(read(run, 'status.csv')) > 20


def nodata(ax, title):
    ax.set_title(title)
    ax.text(.5, .5, 'no data recorded', ha='center', va='center', transform=ax.transAxes)


class Run:
    def __init__(self, path):
        self.path = path.rstrip('/')
        self.odom, self.scan = read(path, 'odom.csv'), read(path, 'scan.csv')
        self.status, self.gas = read(path, 'status.csv'), read(path, 'gas.csv')
        self.det, self.alerts = read(path, 'detections.csv'), read(path, 'alerts.csv')
        self.det = [d for d in self.det if d.get('identity') in KNOWN]


def p_path(ax, R):
    if not R.odom:
        return nodata(ax, 'Robot path')
    x, y, t = col(R.odom, 'x'), col(R.odom, 'y'), col(R.odom, 't')
    sc = ax.scatter(x, y, c=t, s=5, cmap='viridis')
    ax.plot(x[0], y[0], 'go', ms=10, label='start')
    ax.plot(x[-1], y[-1], 'rs', ms=10, label='end')
    ax.set_title('Robot path (colour = time)'); ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)')
    ax.axis('equal'); ax.legend(); ax.figure.colorbar(sc, ax=ax, label='time (s)')


def p_speed(ax, R):
    if not R.odom:
        return nodata(ax, 'Speed')
    ax.plot(col(R.odom, 't'), col(R.odom, 'speed'), lw=1)
    ax.set_title('Robot forward speed'); ax.set_xlabel('time (s)'); ax.set_ylabel('m/s'); ax.grid(alpha=.3)


def p_clear(ax, R):
    if not R.scan:
        return nodata(ax, 'Obstacle distance')
    key = 'min_distance' if 'min_distance' in R.scan[0] else 'min_range'
    ax.plot(col(R.scan, 't'), col(R.scan, key), lw=1)
    ax.axhline(OBSTACLE, color='r', ls='--', label=f'stop threshold {OBSTACLE} m')
    ax.set_title('Nearest obstacle distance (LiDAR)'); ax.set_xlabel('time (s)'); ax.set_ylabel('metres')
    ax.legend(); ax.grid(alpha=.3)


def p_gas(ax, R):
    if not R.gas:
        return nodata(ax, 'Gas level')
    t, ppm = col(R.gas, 't'), col(R.gas, 'ppm')
    ax.plot(t, ppm, color='darkorange')
    for lvl, c in (('WARNING', 'gold'), ('DANGER', 'red'), ('CRITICAL', 'purple')):
        vals = [p for p, g in zip(ppm, R.gas) if g.get('level', '').upper() == lvl and not math.isnan(p)]
        if vals:
            ax.axhline(min(vals), color=c, ls='--', label=f'{lvl} starts ~{min(vals):.0f} ppm')
    ax.set_title('Gas concentration'); ax.set_xlabel('time (s)'); ax.set_ylabel('ppm')
    ax.grid(alpha=.3); ax.legend(fontsize=8)


def p_state(ax, R):
    if not R.status:
        return nodata(ax, 'Robot state')
    states = list(dict.fromkeys(s['state'] for s in R.status))
    ax.step(col(R.status, 't'), [states.index(s['state']) for s in R.status], where='post')
    ax.set_yticks(range(len(states))); ax.set_yticklabels(states, fontsize=8)
    for a in R.alerts:
        try:
            ax.axvline(float(a['t']), color='r', alpha=.5, ls=':')
        except (KeyError, ValueError):
            pass
    ax.set_title('Decision state over time (red = alert)'); ax.set_xlabel('time (s)'); ax.grid(alpha=.3)


def p_det(ax, R):
    if not R.det:
        return nodata(ax, 'Detections')
    ids = [i for i in KNOWN if any(d['identity'] == i for d in R.det)]
    for d in R.det:
        ax.scatter(float(d['t']), ids.index(d['identity']), s=8,
                   c='tab:red' if d['identity'] == 'OUTSIDER' else 'tab:green')
    ax.set_yticks(range(len(ids))); ax.set_yticklabels(ids)
    ax.set_title('Who the robot saw, over time (green = family, red = outsider)')
    ax.set_xlabel('time (s)'); ax.grid(alpha=.3)


def p_counts(ax, R):
    if not R.det:
        return nodata(ax, 'Detection counts')
    names = [i for i in KNOWN if any(d['identity'] == i for d in R.det)]
    counts = [sum(1 for d in R.det if d['identity'] == n) for n in names]
    bars = ax.bar(names, counts, color=['tab:red' if n == 'OUTSIDER' else 'tab:green' for n in names])
    ax.bar_label(bars); ax.set_title('Detections per person'); ax.set_ylabel('count')


PANELS = [('path', p_path), ('speed', p_speed), ('clearance', p_clear),
          ('gas', p_gas), ('state', p_state), ('detections', p_det)]


def summary(R):
    L = [f'Run: {os.path.basename(R.path)}']
    if R.odom:
        x, y, t = col(R.odom, 'x'), col(R.odom, 'y'), col(R.odom, 't')
        dist = sum(math.hypot(x[i + 1] - x[i], y[i + 1] - y[i]) for i in range(len(x) - 1))
        sp = [abs(v) for v in col(R.odom, 'speed') if not math.isnan(v)]
        L += [f'Duration            : {t[-1]-t[0]:.0f} s', f'Distance travelled  : {dist:.1f} m',
              f'Area covered        : x {min(x):.1f}..{max(x):.1f} m, y {min(y):.1f}..{max(y):.1f} m',
              f'Avg / max speed     : {sum(sp)/len(sp):.2f} / {max(sp):.2f} m/s']
    if R.scan:
        key = 'min_distance' if 'min_distance' in R.scan[0] else 'min_range'
        d = [v for v in col(R.scan, key) if not math.isnan(v)]
        L += [f'Closest obstacle    : {min(d):.2f} m',
              f'Readings < {OBSTACLE} m  : {sum(v < OBSTACLE for v in d)} of {len(d)}']
    if R.gas:
        g = [v for v in col(R.gas, 'ppm') if not math.isnan(v)]
        L.append(f'Gas ppm min/avg/max : {min(g):.0f} / {sum(g)/len(g):.0f} / {max(g):.0f}')
        lv = {}
        for r in R.gas:
            lv[r.get('level', '?')] = lv.get(r.get('level', '?'), 0) + 1
        L.append('Gas level readings  : ' + ', '.join(f'{k}={v}' for k, v in lv.items()))
    if R.status:
        L.append('States seen         : ' + ', '.join(dict.fromkeys(s['state'] for s in R.status)))
    if R.det:
        c = {}
        for d in R.det:
            c[d['identity']] = c.get(d['identity'], 0) + 1
        L.append('People detected     : ' + ', '.join(f'{k}={v}' for k, v in c.items()))
    if R.alerts:
        c = {}
        for a in R.alerts:
            k = f"{a.get('type','?')} ({a.get('severity','?')})"
            c[k] = c.get(k, 0) + 1
        L.append(f'Alerts total        : {len(R.alerts)}')
        L += [f'   {k}: {v}' for k, v in c.items()]
    else:
        L.append('Alerts total        : 0')
    return '\n'.join(L)


def make(path):
    R = Run(path)
    out = os.path.join(R.path, 'graphs'); os.makedirs(out, exist_ok=True)
    fig, axs = plt.subplots(2, 3, figsize=(21, 11))
    for ax, (_, fn) in zip(axs.flat, PANELS):
        fn(ax, R)
    fig.suptitle(f'Smart Home Monitoring Robot - {os.path.basename(R.path)}', fontsize=16)
    fig.tight_layout(); fig.savefig(os.path.join(out, 'dashboard.png'), dpi=130); plt.close(fig)
    for name, fn in PANELS + [('detection_counts', p_counts)]:
        f, a = plt.subplots(figsize=(9, 6)); fn(a, R); f.tight_layout()
        f.savefig(os.path.join(out, f'{name}.png'), dpi=130); plt.close(f)
    text = summary(R)
    open(os.path.join(out, 'summary.txt'), 'w').write(text + '\n')
    print('\n' + text + f'\n-> graphs saved in {out}\n')


runs = args or [d for d in sorted(glob.glob(os.path.join(ROOT, '*/')), key=os.path.getmtime, reverse=True)
                if has_data(d)][:5]
if not runs:
    sys.exit(f'No run folder with data found in {ROOT}')
for r in runs:
    make(r)
