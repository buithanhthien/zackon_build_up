#!/usr/bin/env python3
from datetime import datetime

log_file = '/home/khoaiuh/zackon_build_up/tool/debug_controller/cmd_vel_log_3.txt'

prev_time = None
with open(log_file, 'r') as f:
    for line_num, line in enumerate(f, 1):
        line = line.strip()
        if not line:
            continue
            
        timestamp_str = line.split(',')[0]
        current_time = datetime.strptime(timestamp_str, '%Y-%m-%d %H:%M:%S.%f')
        
        if 'linear.x: 0.000' in line and 'angular.z: 0.000' in line:
            print(f"Line {line_num}: {line}")
        
        if prev_time:
            diff_ms = (current_time - prev_time).total_seconds() * 1000
            if diff_ms > 100:
                print(f"Line {line_num}: Gap {diff_ms:.1f}ms from previous message")
        
        prev_time = current_time
