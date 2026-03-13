#!/usr/bin/env python3

import argparse
import csv
import os

import matplotlib.pyplot as plt


def load_rows(csv_path):

    with open(csv_path, newline='', encoding='utf-8') as csvfile:
        rows = list(csv.DictReader(csvfile))

    if not rows:
        raise ValueError('CSV file is empty.')

    times = [float(row['time_sec']) for row in rows]
    t0 = times[0]

    return {
        'time_sec': [t - t0 for t in times],
        'x': [float(row['x']) for row in rows],
        'y': [float(row['y']) for row in rows],
        'ref_x': [float(row['ref_x']) for row in rows],
        'ref_y': [float(row['ref_y']) for row in rows],
        'path_error_m': [float(row['path_error_m']) for row in rows],
        'heading_error_deg': [float(row['heading_error_deg']) for row in rows],
        'linear_velocity_mps': [float(row['linear_velocity_mps']) for row in rows],
        'angular_velocity_radps': [float(row['angular_velocity_radps']) for row in rows],
    }


def save_plot(x, y, title, ylabel, output_path, color):

    plt.figure(figsize=(9, 4.5))
    plt.plot(x, y, color=color, linewidth=2)
    plt.title(title)
    plt.xlabel('Time [s]')
    plt.ylabel(ylabel)
    plt.grid(True, linestyle='--', alpha=0.4)
    plt.tight_layout()
    plt.savefig(output_path, dpi=160)
    plt.close()


def save_xy_plot(actual_x, actual_y, ref_x, ref_y, output_path):

    plt.figure(figsize=(6.5, 6.5))
    plt.plot(ref_x, ref_y, linewidth=2, color='tab:cyan', label='Reference trajectory')
    plt.plot(actual_x, actual_y, linewidth=2, color='tab:red', label='Actual trajectory')
    plt.title('Reference vs Actual Trajectory')
    plt.xlabel('X [m]')
    plt.ylabel('Y [m]')
    plt.axis('equal')
    plt.grid(True, linestyle='--', alpha=0.4)
    plt.legend()
    plt.tight_layout()
    plt.savefig(output_path, dpi=160)
    plt.close()


def main():

    parser = argparse.ArgumentParser(
        description='Generate navigation metric plots from a CSV log.'
    )
    parser.add_argument(
        'csv_path',
        help='Path to navigation_metrics.csv'
    )
    parser.add_argument(
        '--output-dir',
        default='/home/bim/origin_ws/results/navigation_plots',
        help='Directory for PNG plot files'
    )
    args = parser.parse_args()

    data = load_rows(args.csv_path)
    os.makedirs(args.output_dir, exist_ok=True)

    save_plot(
        data['time_sec'],
        data['path_error_m'],
        'Path Tracking Error',
        'Error [m]',
        os.path.join(args.output_dir, 'path_error.png'),
        'tab:red'
    )
    save_plot(
        data['time_sec'],
        data['heading_error_deg'],
        'Heading Error',
        'Error [deg]',
        os.path.join(args.output_dir, 'heading_error.png'),
        'tab:orange'
    )
    save_plot(
        data['time_sec'],
        data['linear_velocity_mps'],
        'Linear Velocity',
        'Velocity [m/s]',
        os.path.join(args.output_dir, 'linear_velocity.png'),
        'tab:blue'
    )
    save_plot(
        data['time_sec'],
        data['angular_velocity_radps'],
        'Angular Velocity',
        'Velocity [rad/s]',
        os.path.join(args.output_dir, 'angular_velocity.png'),
        'tab:green'
    )
    save_xy_plot(
        data['x'],
        data['y'],
        data['ref_x'],
        data['ref_y'],
        os.path.join(args.output_dir, 'trajectory_xy.png')
    )

    print(f'Plots saved to {args.output_dir}')


if __name__ == '__main__':
    main()
