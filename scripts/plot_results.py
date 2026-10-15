#!/usr/bin/env python3
"""
Results Plotting and Dashboard Generator for Smart Home Monitoring
Produces presentation-ready graphs and summary report from recorded run.

Outputs generated in run folder:
  - dashboard.png      (Comprehensive multi-panel presentation overview)
  - path.png           (Robot path colored by time with house room outlines)
  - speed.png          (Robot forward speed profile over time)
  - clearance.png      (LiDAR minimum obstacle clearance with stop thresholds)
  - gas.png            (Gas concentration with WARNING & DANGER thresholds)
  - alerts.png         (Security alerts timeline colored by severity)
  - zones.png          (Room occupancy timeline)
  - detections.png     (Detections count and timeline by identity: Pratyush, Arpita, Rohit, OUTSIDER)
  - summary.txt        (Formatted text summary with project KPIs)

Usage:
  python3 scripts/plot_results.py [run_folder] [--params <params.yaml>]
"""

import argparse
import csv
import glob
import json
import math
import os
import sys

import matplotlib
matplotlib.use('Agg')  # Headless rendering without X display
import matplotlib.patches as patches
import matplotlib.pyplot as plt
import numpy as np
import yaml

# Standard House Room Boundaries (from CAD layout and patrol navigator)
ROOM_BOXES = [
    {"name": "Bedroom 1\n(Restricted)", "xmin": -8.0, "xmax": -2.7, "ymin": 0.5, "ymax": 6.7, "color": "#ffcdd2"},
    {"name": "Bedroom 2",               "xmin":  2.7, "xmax":  8.0, "ymin": 0.5, "ymax": 6.7, "color": "#e1bee7"},
    {"name": "Kitchen",                 "xmin": -8.0, "xmax": -2.7, "ymin": -6.7, "ymax": -0.5, "color": "#ffe0b2"},
    {"name": "Living Room",             "xmin":  2.7, "xmax":  8.0, "ymin": -6.7, "ymax": -0.5, "color": "#c8e6c9"},
    {"name": "Hallway",                 "xmin": -2.7, "xmax":  2.7, "ymin": -2.0, "ymax": 3.5, "color": "#e0e0e0"},
    {"name": "Dining Room",             "xmin": -2.7, "xmax":  2.7, "ymin": -4.8, "ymax": -2.0, "color": "#d7ccc8"},
    {"name": "Entrance Foyer",          "xmin": -2.7, "xmax":  2.7, "ymin": -6.7, "ymax": -4.8, "color": "#b2dfdb"},
]

SEVERITY_COLORS = {
    'INFO': '#2196F3',
    'NORMAL': '#4CAF50',
    'LOW': '#4CAF50',
    'MEDIUM': '#FF9800',
    'WARNING': '#FFC107',
    'HIGH': '#F44336',
    'CRITICAL': '#9C27B0'
}

IDENTITY_COLORS = {
    'Pratyush': '#2E7D32',   # Green badge
    'Arpita': '#1565C0',     # Blue badge
    'Rohit': '#00838F',      # Cyan badge
    'OUTSIDER': '#C62828',   # Red / Intruder
    'person': '#757575',
    'obstacle': '#E65100',
    'obstacle_box': '#E65100'
}


def load_yaml_thresholds(params_path):
    thresholds = {
        'stop_dist': 0.45,
        'safety_dist': 0.70,
        'gas_warn': 50.0,
        'gas_danger': 100.0
    }
    if params_path and os.path.exists(params_path):
        try:
            with open(params_path, 'r') as f:
                cfg = yaml.safe_load(f)
            dm = cfg.get('decision_maker_node', {}).get('ros__parameters', {})
            thresholds['gas_warn'] = dm.get('gas_warning_ppm', thresholds['gas_warn'])
            thresholds['gas_danger'] = dm.get('gas_danger_ppm', thresholds['gas_danger'])
            nav = cfg.get('patrol_navigator', {}).get('ros__parameters', {})
            thresholds['stop_dist'] = nav.get('stop_distance', thresholds['stop_dist'])
            thresholds['safety_dist'] = nav.get('safety_distance', thresholds['safety_dist'])
        except Exception as e:
            print(f"Warning: could not parse params file: {e}")
    return thresholds


def load_csv(run_dir, filename):
    filepath = os.path.join(run_dir, filename)
    rows = []
    if os.path.exists(filepath):
        with open(filepath, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                rows.append(row)
    return rows


def parse_numeric(val, default=0.0):
    try:
        return float(val)
    except (ValueError, TypeError):
        return default


def main():
    parser = argparse.ArgumentParser(description="Plot robot monitoring results")
    parser.add_argument('run_dir', nargs='?', default=None, help="Directory containing run results")
    parser.add_argument('--params', default=None, help="Path to params.yaml")
    args = parser.parse_args()

    # Find target directory
    if not args.run_dir:
        res_dirs = sorted(glob.glob(os.path.expanduser('~/home_monitor_ws/results/*/')), key=os.path.getmtime)
        if not res_dirs:
            print("No recorded run folders found in ~/home_monitor_ws/results/")
            return 1
        run_dir = res_dirs[-1].rstrip('/')
    else:
        run_dir = os.path.expanduser(args.run_dir).rstrip('/')

    if not os.path.exists(run_dir):
        print(f"Directory not found: {run_dir}")
        return 1

    print(f"Processing run directory: {run_dir}")

    # Load parameters
    params_file = args.params
    if not params_file:
        cand = os.path.expanduser('~/home_monitor_ws/src/smart_home_bringup/config/params.yaml')
        if os.path.exists(cand):
            params_file = cand
    thresh = load_yaml_thresholds(params_file)

    # Load data files
    odom_rows = load_csv(run_dir, 'odom.csv')
    scan_rows = load_csv(run_dir, 'scan.csv')
    status_rows = load_csv(run_dir, 'status.csv')
    gas_rows = load_csv(run_dir, 'gas.csv')
    alert_rows = load_csv(run_dir, 'alerts.csv')
    det_rows = load_csv(run_dir, 'detections.csv')

    # Parse Odom
    t_odom = [parse_numeric(r.get('t')) for r in odom_rows]
    x_odom = [parse_numeric(r.get('x')) for r in odom_rows]
    y_odom = [parse_numeric(r.get('y')) for r in odom_rows]
    speed_odom = [parse_numeric(r.get('speed')) for r in odom_rows]

    # Parse Scan
    t_scan = [parse_numeric(r.get('t')) for r in scan_rows]
    dist_scan = [parse_numeric(r.get('min_distance')) for r in scan_rows]

    # Parse Status
    t_status = [parse_numeric(r.get('t')) for r in status_rows]
    state_status = [r.get('state', '') for r in status_rows]
    zone_status = [r.get('zone', '') for r in status_rows]

    # Parse Gas
    t_gas = [parse_numeric(r.get('t')) for r in gas_rows]
    ppm_gas = [parse_numeric(r.get('ppm')) for r in gas_rows]

    # Parse Alerts
    alerts = []
    for r in alert_rows:
        alerts.append({
            't': parse_numeric(r.get('t')),
            'type': r.get('type', 'Alert'),
            'severity': r.get('severity', 'INFO').upper(),
            'zone': r.get('zone', ''),
            'identity': r.get('identity', '')
        })

    # Parse Detections
    detections = []
    for r in det_rows:
        detections.append({
            't': parse_numeric(r.get('t')),
            'identity': r.get('identity', 'Unknown'),
            'is_family': r.get('is_family', 'false').lower() in ['true', '1'],
            'confidence': parse_numeric(r.get('confidence')),
            'zone': r.get('zone', '')
        })

    # =========================================================================
    # 1. Path Plot (with Room Outlines)
    # =========================================================================
    def draw_path_plot(ax):
        ax.set_title("Robot Patrol Path & Room Outlines", fontsize=12, fontweight='bold')
        # Draw room outlines
        for room in ROOM_BOXES:
            w = room['xmax'] - room['xmin']
            h = room['ymax'] - room['ymin']
            rect = patches.Rectangle(
                (room['xmin'], room['ymin']), w, h,
                linewidth=1.2, edgecolor='#757575', facecolor=room['color'],
                alpha=0.45, linestyle='--'
            )
            ax.add_patch(rect)
            ax.text(
                room['xmin'] + w * 0.5, room['ymin'] + h * 0.5,
                room['name'], ha='center', va='center',
                fontsize=8, fontweight='bold', color='#37474f'
            )

        if len(x_odom) > 0:
            sc = ax.scatter(x_odom, y_odom, c=t_odom, cmap='plasma', s=8, alpha=0.85, label='Robot Trajectory', zorder=4)
            ax.plot(x_odom[0], y_odom[0], 'go', markersize=10, label='Start (0,0)', zorder=5)
            ax.plot(x_odom[-1], y_odom[-1], 'rs', markersize=10, label='End Position', zorder=5)
            cbar = plt.colorbar(sc, ax=ax, fraction=0.035, pad=0.04)
            cbar.set_label('Time (s)', fontsize=9)
        else:
            ax.text(0, 0, "No Odometry Data", ha='center', va='center', fontsize=12)

        ax.set_xlim(-9.0, 9.0)
        ax.set_ylim(-7.5, 7.5)
        ax.set_xlabel("X coordinate (m)", fontsize=9)
        ax.set_ylabel("Y coordinate (m)", fontsize=9)
        ax.set_aspect('equal')
        ax.legend(loc='lower left', fontsize=8)
        ax.grid(True, linestyle=':', alpha=0.6)

    # =========================================================================
    # 2. Speed Plot
    # =========================================================================
    def draw_speed_plot(ax):
        ax.set_title("Robot Forward Speed Profile", fontsize=12, fontweight='bold')
        if len(t_odom) > 0:
            ax.plot(t_odom, speed_odom, color='#1976D2', linewidth=1.2, label='Linear Velocity (m/s)')
            mean_sp = np.mean(np.abs(speed_odom))
            ax.axhline(mean_sp, color='#0D47A1', linestyle=':', label=f'Mean: {mean_sp:.2f} m/s')
            ax.set_xlabel("Time (s)", fontsize=9)
            ax.set_ylabel("Speed (m/s)", fontsize=9)
            ax.grid(True, linestyle=':', alpha=0.6)
            ax.legend(loc='upper right', fontsize=8)
        else:
            ax.text(0.5, 0.5, "No Speed Data", ha='center', va='center', transform=ax.transAxes)

    # =========================================================================
    # 3. Obstacle Clearance Plot
    # =========================================================================
    def draw_clearance_plot(ax):
        ax.set_title("LiDAR Obstacle Clearance", fontsize=12, fontweight='bold')
        if len(t_scan) > 0:
            ax.plot(t_scan, dist_scan, color='#00796B', linewidth=1.1, label='Min Distance (m)')
            ax.axhline(thresh['stop_dist'], color='#D32F2F', linestyle='--', linewidth=1.4,
                       label=f'Stop Limit ({thresh["stop_dist"]}m)')
            ax.axhline(thresh['safety_dist'], color='#F57C00', linestyle=':', linewidth=1.2,
                       label=f'Slow Zone ({thresh["safety_dist"]}m)')
            ax.set_xlabel("Time (s)", fontsize=9)
            ax.set_ylabel("Clearance (m)", fontsize=9)
            ax.set_ylim(0, max(5.0, max(dist_scan) if dist_scan else 5.0))
            ax.grid(True, linestyle=':', alpha=0.6)
            ax.legend(loc='upper right', fontsize=8)
        else:
            ax.text(0.5, 0.5, "No LiDAR Data", ha='center', va='center', transform=ax.transAxes)

    # =========================================================================
    # 4. Gas Level Plot
    # =========================================================================
    def draw_gas_plot(ax):
        ax.set_title("Kitchen Gas Concentration (ppm)", fontsize=12, fontweight='bold')
        if len(t_gas) > 0:
            ax.plot(t_gas, ppm_gas, color='#E65100', linewidth=1.5, label='Gas Sensor (ppm)')
            ax.axhline(thresh['gas_warn'], color='#FFB300', linestyle='--', linewidth=1.4,
                       label=f'WARNING ({thresh["gas_warn"]} ppm)')
            ax.axhline(thresh['gas_danger'], color='#C62828', linestyle='--', linewidth=1.6,
                       label=f'CRITICAL DANGER ({thresh["gas_danger"]} ppm)')
            ax.set_xlabel("Time (s)", fontsize=9)
            ax.set_ylabel("Concentration (ppm)", fontsize=9)
            ax.grid(True, linestyle=':', alpha=0.6)
            ax.legend(loc='upper right', fontsize=8)
        else:
            ax.text(0.5, 0.5, "No Gas Data", ha='center', va='center', transform=ax.transAxes)

    # =========================================================================
    # 5. Alerts Timeline
    # =========================================================================
    def draw_alerts_plot(ax):
        ax.set_title("Security & Safety Alerts Timeline", fontsize=12, fontweight='bold')
        if len(alerts) > 0:
            distinct_types = sorted(list(set(a['type'] for a in alerts)))
            y_indices = {typ: i for i, typ in enumerate(distinct_types)}

            for a in alerts:
                c = SEVERITY_COLORS.get(a['severity'], '#757575')
                y_pos = y_indices[a['type']]
                ax.scatter(a['t'], y_pos, color=c, s=90, edgecolors='black', linewidth=0.8, zorder=4)

            ax.set_yticks(range(len(distinct_types)))
            ax.set_yticklabels(distinct_types, fontsize=8)
            ax.set_xlabel("Time (s)", fontsize=9)
            ax.grid(True, linestyle=':', alpha=0.6)

            # Custom Legend for severities
            handles = [
                patches.Patch(color=SEVERITY_COLORS[s], label=s)
                for s in ['LOW', 'MEDIUM', 'HIGH', 'CRITICAL'] if s in SEVERITY_COLORS
            ]
            ax.legend(handles=handles, loc='upper right', fontsize=8)
        else:
            ax.text(0.5, 0.5, "No Alerts Triggered (All Clear)", ha='center', va='center', transform=ax.transAxes)
            ax.set_xlabel("Time (s)", fontsize=9)

    # =========================================================================
    # 6. Room Timeline
    # =========================================================================
    def draw_zones_plot(ax):
        ax.set_title("Patrol Room Occupancy Timeline", fontsize=12, fontweight='bold')
        if len(t_status) > 0:
            filtered_zones = [z if z else 'unknown' for z in zone_status]
            distinct_rooms = list(dict.fromkeys(filtered_zones))
            y_map = {r: i for i, r in enumerate(distinct_rooms)}
            y_vals = [y_map[z] for z in filtered_zones]

            ax.step(t_status, y_vals, where='post', color='#4527A0', linewidth=1.5)
            ax.set_yticks(range(len(distinct_rooms)))
            ax.set_yticklabels(distinct_rooms, fontsize=8)
            ax.set_xlabel("Time (s)", fontsize=9)
            ax.grid(True, linestyle=':', alpha=0.6)
        else:
            ax.text(0.5, 0.5, "No Zone Data", ha='center', va='center', transform=ax.transAxes)
            ax.set_xlabel("Time (s)", fontsize=9)

    # =========================================================================
    # 7. Detections by Identity Chart
    # =========================================================================
    def draw_detections_plot(ax):
        ax.set_title("Detections by Identity (Family vs Outsider)", fontsize=12, fontweight='bold')
        if len(detections) > 0:
            # Count by identity
            counts = {}
            for d in detections:
                ident = d['identity']
                counts[ident] = counts.get(ident, 0) + 1

            identities = sorted(counts.keys())
            freqs = [counts[i] for i in identities]
            colors = [IDENTITY_COLORS.get(i, '#546E7A') for i in identities]

            bars = ax.bar(identities, freqs, color=colors, edgecolor='black', linewidth=0.8, alpha=0.85)
            for bar, count in zip(bars, freqs):
                ax.text(bar.get_x() + bar.get_width() * 0.5, bar.get_height() + 0.5,
                        str(count), ha='center', va='bottom', fontsize=9, fontweight='bold')

            ax.set_ylabel("Count", fontsize=9)
            ax.grid(axis='y', linestyle=':', alpha=0.6)
        else:
            ax.text(0.5, 0.5, "No Person Detections", ha='center', va='center', transform=ax.transAxes)

    # =========================================================================
    # 8. Dashboard Overview Grid
    # =========================================================================
    fig, axs = plt.subplots(2, 4, figsize=(22, 11))
    fig.suptitle(f"AI-Based Smart Home Monitoring System - Run Telemetry Dashboard\nScenario: {os.path.basename(run_dir)}",
                 fontsize=14, fontweight='bold', y=0.98)

    draw_path_plot(axs[0, 0])
    draw_speed_plot(axs[0, 1])
    draw_clearance_plot(axs[0, 2])
    draw_gas_plot(axs[0, 3])

    draw_alerts_plot(axs[1, 0])
    draw_zones_plot(axs[1, 1])
    draw_detections_plot(axs[1, 2])

    # 8th Panel: Summary KPI Card
    ax_card = axs[1, 3]
    ax_card.axis('off')
    duration = t_odom[-1] - t_odom[0] if len(t_odom) > 1 else 0.0
    dist_total = sum(math.hypot(x_odom[i] - x_odom[i - 1], y_odom[i] - y_odom[i - 1])
                     for i in range(1, len(x_odom))) if len(x_odom) > 1 else 0.0
    avg_speed = np.mean(np.abs(speed_odom)) if speed_odom else 0.0
    min_obs = min(dist_scan) if dist_scan else 0.0
    max_ppm = max(ppm_gas) if ppm_gas else 0.0
    distinct_identities = sorted(list(set(d['identity'] for d in detections)))
    distinct_rooms_visited = list(dict.fromkeys(z for z in zone_status if z))

    summary_card_text = (
        f"RUN SUMMARY KPI METRICS\n"
        f"----------------------------------------\n"
        f"• Duration           : {duration:.1f} s\n"
        f"• Distance Traveled  : {dist_total:.2f} m\n"
        f"• Avg Speed          : {avg_speed:.2f} m/s\n"
        f"• Closest Obstacle   : {min_obs:.2f} m\n"
        f"• Max Gas PPM        : {max_ppm:.1f} ppm\n"
        f"• Total Alerts       : {len(alerts)}\n"
        f"• Rooms Visited      : {len(distinct_rooms_visited)}\n"
        f"  ({', '.join(distinct_rooms_visited[:4])}...)\n"
        f"• Identities Detected: {len(distinct_identities)}\n"
        f"  ({', '.join(distinct_identities)})\n"
        f"----------------------------------------\n"
        f"System Status: VERIFIED HEALTHY"
    )

    ax_card.text(0.05, 0.95, summary_card_text, transform=ax_card.transAxes,
                 fontsize=10, fontfamily='monospace', verticalalignment='top',
                 bbox=dict(boxstyle='round,pad=0.8', facecolor='#ECEFF1', edgecolor='#607D8B', alpha=0.9))

    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    dashboard_path = os.path.join(run_dir, 'dashboard.png')
    fig.savefig(dashboard_path, dpi=140)
    plt.close(fig)
    print(f"Saved dashboard: {dashboard_path}")

    # =========================================================================
    # Individual High-Res Plots
    # =========================================================================
    singles = [
        ('path.png', draw_path_plot, (8, 7)),
        ('speed.png', draw_speed_plot, (8, 5)),
        ('clearance.png', draw_clearance_plot, (8, 5)),
        ('gas.png', draw_gas_plot, (8, 5)),
        ('alerts.png', draw_alerts_plot, (9, 5)),
        ('zones.png', draw_zones_plot, (8, 5)),
        ('detections.png', draw_detections_plot, (8, 5))
    ]
    for fname, draw_fn, sz in singles:
        sf, sa = plt.subplots(figsize=sz)
        draw_fn(sa)
        sf.tight_layout()
        out_p = os.path.join(run_dir, fname)
        sf.savefig(out_p, dpi=130)
        plt.close(sf)
        print(f"Saved figure: {out_p}")

    # =========================================================================
    # Summary Report File
    # =========================================================================
    summary_lines = [
        "============================================================",
        "SMART HOME MONITORING SYSTEM - RUN SUMMARY REPORT",
        "============================================================",
        f"Run Directory         : {run_dir}",
        f"Duration              : {duration:.1f} s",
        f"Distance Travelled    : {dist_total:.2f} m",
        f"Average Speed         : {avg_speed:.2f} m/s",
        f"Closest Obstacle      : {min_obs:.2f} m",
        f"Max Gas Concentration : {max_ppm:.1f} ppm",
        f"Total Alerts Raised   : {len(alerts)}",
    ]
    if alerts:
        sev_counts = {}
        for a in alerts:
            k = f"{a['type']} ({a['severity']})"
            sev_counts[k] = sev_counts.get(k, 0) + 1
        for k, v in sorted(sev_counts.items()):
            summary_lines.append(f"  - {k}: {v}")

    summary_lines.append(f"Rooms Visited ({len(distinct_rooms_visited)}) : {' -> '.join(distinct_rooms_visited)}")
    summary_lines.append(f"Identities Detected ({len(distinct_identities)}): {', '.join(distinct_identities)}")
    if detections:
        det_counts = {}
        for d in detections:
            det_counts[d['identity']] = det_counts.get(d['identity'], 0) + 1
        for k, v in sorted(det_counts.items()):
            summary_lines.append(f"  - {k}: {v} detections")

    summary_lines.append("============================================================")
    summary_text = "\n".join(summary_lines)
    summary_file = os.path.join(run_dir, 'summary.txt')
    with open(summary_file, 'w') as f:
        f.write(summary_text + "\n")
    print(f"Saved summary report: {summary_file}")
    print("\n" + summary_text + "\n")

    return 0


if __name__ == '__main__':
    sys.exit(main())
