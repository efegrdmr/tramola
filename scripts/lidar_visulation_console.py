#!/usr/bin/env python3
import rospy
import math
import sys
import shutil
from sensor_msgs.msg import LaserScan

def scan_callback(msg):
    # Parameters (can be overridden via ROS params)
    width  = rospy.get_param('~width', 0)
    height = rospy.get_param('~height', 0)
    plot_radius_m = rospy.get_param('~plot_radius_m', 10.0)  # meters represented from center to any edge
    auto_size = rospy.get_param('~auto_size', True)
    invert_y = rospy.get_param('~invert_y', True)  # terminal rows grow downward; invert to have +Y up
    show_axes = rospy.get_param('~show_axes', True)

    # Derive terminal size if requested or if width/height unset
    if auto_size or width <= 0 or height <= 0:
        try:
            term_size = shutil.get_terminal_size(fallback=(80, 40))
            # Leave a few columns/rows for prompts
            width = max(20, min(term_size.columns - 2, 200))
            height = max(10, min(term_size.lines - 4, 100))
        except Exception:
            width, height = 80, 40

    # Guard against division by zero
    if width < 4 or height < 4:
        return

    # Create empty grid
    grid = [[' ' for _ in range(width)] for _ in range(height)]
    center_x = width // 2
    center_y = height // 2

    # Optional axes
    if show_axes:
        for x in range(width):
            grid[center_y][x] = '-'
        for y in range(height):
            grid[y][center_x] = '|'
    grid[center_y][center_x] = 'O'  # robot position

    # Precompute scaling factors
    half_w = (width - 2) / 2.0
    half_h = (height - 2) / 2.0
    max_plot_r = plot_radius_m

    angle = msg.angle_min
    for r in msg.ranges:
        if not math.isfinite(r) or r <= 0.0:
            angle += msg.angle_increment
            continue
        # Skip points outside our chosen radius
        if r > max_plot_r:
            angle += msg.angle_increment
            continue
        # Convert polar to Cartesian
        x_m = r * math.cos(angle)
        y_m = r * math.sin(angle)
        # Scale to grid
        x = int(center_x + (x_m / max_plot_r) * half_w)
        y_offset = (y_m / max_plot_r) * half_h
        if invert_y:
            y = int(center_y - y_offset)
        else:
            y = int(center_y + y_offset)
        if 0 <= x < width and 0 <= y < height:
            # Preserve center marker and axes when overwriting
            if grid[y][x] in ('O', '|', '-'):
                grid[y][x] = '+' if grid[y][x] != 'O' else 'O'
            else:
                grid[y][x] = '*'
        angle += msg.angle_increment

    # Clear screen & reset cursor (ANSI)
    sys.stdout.write("\033[H\033[J")

    # Render
    for row in grid:
        sys.stdout.write(''.join(row) + "\n")
    sys.stdout.write(f"Radius: {max_plot_r}m  Points: {len(msg.ranges)}  (Ctrl+C to quit)\n")
    sys.stdout.flush()


def main():
    rospy.init_node('scan_ascii_viz')
    # New param for topic name
    scan_topic = rospy.get_param('~scan_topic', '/scan_filtered')
    rospy.Subscriber(scan_topic, LaserScan, scan_callback, queue_size=1)
    rospy.loginfo(f"scan_ascii_viz started. Subscribed to {scan_topic}. Publishing ASCII LIDAR visualization to console.")
    rospy.spin()

if __name__ == '__main__':
    main()
