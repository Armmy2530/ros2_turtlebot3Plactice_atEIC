#!/usr/bin/env python3

import os
import time
import psutil
import threading
from concurrent.futures import ThreadPoolExecutor
import subprocess
from collections import namedtuple
import platform

# Create a named tuple for process info
ProcessInfo = namedtuple('ProcessInfo', ['pid', 'name', 'cpu', 'ram_gb'])

def clear_screen():
    """Clear the terminal screen."""
    if platform.system() == "Windows":
        os.system('cls')
    else:
        os.system('clear')

# Add at the top of the file, after the imports:
process_cpu_times = {}

# Replace the get_process_info function with:
def get_process_info(pid):
    """Get information for a single process"""
    try:
        process = psutil.Process(pid)
        with process.oneshot():
            # Get CPU usage
            cpu_percent = 0
            if pid in process_cpu_times:
                # Calculate CPU usage since last measurement
                current_time = time.time()
                current_cpu_time = sum(process.cpu_times()[:2])  # User + System time
                time_delta = current_time - process_cpu_times[pid]['time']
                cpu_time_delta = current_cpu_time - process_cpu_times[pid]['cpu_time']
                cpu_percent = (cpu_time_delta / time_delta) * 100

            # Store current measurements for next iteration
            process_cpu_times[pid] = {
                'time': time.time(),
                'cpu_time': sum(process.cpu_times()[:2])
            }

            return ProcessInfo(
                pid=pid,
                name=process.name(),
                cpu=cpu_percent,
                ram_gb=process.memory_info().rss / (1024 * 1024 * 1024)
            )
    except (psutil.NoSuchProcess, psutil.AccessDenied):
        if pid in process_cpu_times:
            del process_cpu_times[pid]
        return None

def get_ros_processes():
    """Get list of ROS process PIDs"""
    try:
        output = subprocess.check_output(['pgrep', '-f', 'ros']).decode()
        return [int(pid) for pid in output.split()]
    except subprocess.CalledProcessError:
        return []

def main():
    while True:
        # Clear screen
        clear_screen()

        # Get system memory info
        total_memory_gb = psutil.virtual_memory().total / (1024 * 1024 * 1024)

        print("=== ROS Process Resource Usage === (Press Ctrl+C to exit)\n")
        print("Finding ROS processes...")

        ros_pids = get_ros_processes()
        if not ros_pids:
            print("No ROS processes found running.")
            time.sleep(1)
            continue

        # Print header
        print("\n{:<6} {:<20} {:<10} {:<15}".format("PID", "COMMAND", "CPU%", "RAM (GB)"))
        print("-" * 51)

        # Use ThreadPoolExecutor for concurrent processing
        process_infos = []
        with ThreadPoolExecutor(max_workers=min(32, len(ros_pids))) as executor:
            # Submit all processes to the executor
            future_to_pid = {executor.submit(get_process_info, pid): pid for pid in ros_pids}
            
            # Collect results as they complete
            for future in future_to_pid:
                info = future.result()
                if info:
                    process_infos.append(info)

        # Sort by PID and display results
        total_cpu = 0
        total_ram_gb = 0

        for info in sorted(process_infos, key=lambda x: x.pid):
            print(f"{info.pid:<6} {info.name:<20} {info.cpu:<10.1f} {info.ram_gb:<15.3f}")
            total_cpu += info.cpu
            total_ram_gb += info.ram_gb

        # Print totals
        print(f"\nSystem Total RAM: {total_memory_gb:.1f} GB")
        print(f"Total Usage -> CPU: {total_cpu:.1f}% | RAM: {total_ram_gb:.3f} GB")

        time.sleep(1)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nExiting...")