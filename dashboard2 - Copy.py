import socket
import tkinter as tk
from tkinter import scrolledtext
import threading
import math
import time
import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.gridspec as gridspec

# --- GLOBAL DATA STORAGE ---
cybot_socket = None
window = None
canvas = None

absolute_path = os.path.dirname(os.path.abspath(__file__))
raw_file_path = os.path.join(absolute_path, 'sensor-scan.txt')
clean_file_path = os.path.join(absolute_path, 'cleaned-scan.txt')

# Telemetry / Odometry History
robot_x, robot_y, robot_heading = 0.0, 0.0, 90
path_x, path_y = [0.0], [0.0]

# Radar Sweep Data
angles_deg, ping_distances, ir_raw_vals = [], [], []
current_scan_objects = [] # Stores active highlights for the radar graphs

# Matplotlib Objects
ax_map = None
ax_polar = None
ax_linear = None
path_line = None
robot_marker = None
robot_heading_line = None

# UI StringVars for Status Panel
stat_pos, stat_head, stat_cliff_l, stat_cliff_r, stat_bump = None, None, None, None, None

# --- GUI LAYOUT & SETUP ---
def main():
    global window, canvas, telemetry_text
    global stat_pos, stat_head, stat_cliff_l, stat_cliff_r, stat_bump
    global ax_map, ax_polar, ax_linear
    global path_line, robot_marker, robot_heading_line

    window = tk.Tk()
    window.title("CyBot Advanced Logistics Dashboard")
    window.geometry("1600x950")

    # StringVars
    stat_pos = tk.StringVar(value="Robot X: 0.0 cm | Y: 0.0 cm")
    stat_head = tk.StringVar(value="Robot Angle: 90°")
    stat_cliff_l = tk.StringVar(value="Cliff Left: 0 | Front Left: 0")
    stat_cliff_r = tk.StringVar(value="Cliff Right: 0 | Front Right: 0")
    stat_bump = tk.StringVar(value="Bump Left: 0 | Right: 0")

    # --- LEFT PANEL: Controls & Telemetry ---
    control_frame = tk.Frame(window, padx=15, pady=15, width=400)
    control_frame.pack(side=tk.LEFT, fill=tk.Y)

    # 1. LIVE SENSOR STATUS
    status_frame = tk.LabelFrame(control_frame, text="Live Sensor Status", font=("Arial", 10, "bold"), fg="#d32f2f", padx=10, pady=10)
    status_frame.pack(fill=tk.X, pady=(0, 10))
    tk.Label(status_frame, textvariable=stat_pos, font=("Courier", 10), fg="#d32f2f", anchor="w").pack(fill=tk.X)
    tk.Label(status_frame, textvariable=stat_head, font=("Courier", 10), fg="#d32f2f", anchor="w").pack(fill=tk.X)
    tk.Label(status_frame, textvariable=stat_cliff_l, font=("Courier", 10), fg="#d32f2f", anchor="w").pack(fill=tk.X)
    tk.Label(status_frame, textvariable=stat_cliff_r, font=("Courier", 10), fg="#d32f2f", anchor="w").pack(fill=tk.X)
    tk.Label(status_frame, textvariable=stat_bump, font=("Courier", 10), fg="#d32f2f", anchor="w").pack(fill=tk.X)

    # 2. TELEMETRY TERMINAL 
    tk.Label(control_frame, text="COMMUNICATIONS TERMINAL", font=("Arial", 10, "bold")).pack(pady=(5,0))
    telemetry_text = scrolledtext.ScrolledText(control_frame, width=45, height=12, font=("Courier", 10), bg="#1e1e1e", fg="#00ff00")
    telemetry_text.pack(pady=5)
    telemetry_text.insert(tk.END, "Waiting for connection to base station...\n")
    telemetry_text.config(state=tk.DISABLED)

    # 3. MANUAL CONTROLS
    tk.Label(control_frame, text="MANUAL OVERRIDE (WASD)", font=("Arial", 10, "bold")).pack(pady=(10,5))
    btn_frame1 = tk.Frame(control_frame)
    btn_frame1.pack()
    tk.Button(btn_frame1, text="Forward (W)", width=12, command=lambda: send_cmd('w')).grid(row=0, column=1, pady=4)
    tk.Button(btn_frame1, text="Left (Q)", width=12, command=lambda: send_cmd('q')).grid(row=1, column=0, padx=4)
    tk.Button(btn_frame1, text="Reverse (S)", width=12, command=lambda: send_cmd('s')).grid(row=1, column=1)
    tk.Button(btn_frame1, text="Right (E)", width=12, command=lambda: send_cmd('e')).grid(row=1, column=2, padx=4)

    # 4. ADVANCED & ARGUMENT CONTROLS
    tk.Label(control_frame, text="ADVANCED & PARAMETERS", font=("Arial", 10, "bold")).pack(pady=(15,5))
    adv_frame = tk.Frame(control_frame)
    adv_frame.pack()
    
    tk.Label(adv_frame, text="Value (mm/deg):", font=("Arial", 9)).grid(row=0, column=0, padx=2, sticky="e")
    val_entry = tk.Entry(adv_frame, width=10)
    val_entry.grid(row=0, column=1, padx=2, pady=5, sticky="w")
    val_entry.insert(0, "150")

    def send_arg_cmd(cmd_char):
        val = val_entry.get()
        if cybot_socket:
            def task():
                try:
                    cybot_socket.sendall(cmd_char.encode())
                    time.sleep(0.3) 
                    for c in val:
                        cybot_socket.sendall(c.encode())
                        time.sleep(0.05)
                    cybot_socket.sendall(b'\r')
                    window.after(0, log_terminal, f"> Sent: [{cmd_char}] with value [{val}]")
                except Exception as e:
                    window.after(0, log_terminal, f"! Error sending: {e}")
            threading.Thread(target=task, daemon=True).start()

    tk.Button(adv_frame, text="Move Dist (f)", width=12, command=lambda: send_arg_cmd('f')).grid(row=1, column=0, pady=2, padx=2)
    tk.Button(adv_frame, text="Turn Angle (t)", width=12, command=lambda: send_arg_cmd('t')).grid(row=1, column=1, pady=2, padx=2)
    tk.Button(adv_frame, text="Sweep (g)", width=12, command=lambda: send_cmd('g')).grid(row=2, column=0, pady=2, padx=2)
    tk.Button(adv_frame, text="Calibrate (c)", width=12, command=lambda: send_cmd('c')).grid(row=2, column=1, pady=2, padx=2)
    tk.Button(adv_frame, text="Clean Data", width=12, command=clean_data_action, bg="yellow").grid(row=3, column=0, columnspan=2, pady=5)
    # 5. MISSION MODES
    tk.Label(control_frame, text="MISSION MODES", font=("Arial", 10, "bold")).pack(pady=(15,5))
    btn_frame2 = tk.Frame(control_frame)
    btn_frame2.pack(pady=5)
    tk.Button(btn_frame2, text="MANUAL MODE (M)", width=18, height=2, bg="lightblue", command=lambda: send_cmd('m')).grid(row=0, column=0, padx=5)
    tk.Button(btn_frame2, text="AUTO MISSION (A)", width=18, height=2, bg="lightgreen", command=lambda: send_cmd('a')).grid(row=0, column=1, padx=5)

    tk.Button(control_frame, text="EMERGENCY STOP (SPACE)", width=38, height=2, bg="red", fg="white", font=("Arial", 11, "bold"), command=lambda: send_cmd('x')).pack(pady=10)
    tk.Button(control_frame, text="RESET / IDLE (R)", width=38, height=2, bg="orange", command=lambda: send_cmd('r')).pack(pady=5)
    tk.Button(control_frame, text="Disconnect & Quit", width=20, command=send_quit).pack(side=tk.BOTTOM, pady=10)

    # --- RIGHT PANEL: Tri-Graph Layout ---
    plot_frame = tk.Frame(window)
    plot_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

    fig = plt.figure(figsize=(12, 10))
    fig.patch.set_facecolor('#ffffff')
    
    # Create 2x2 Grid, Map takes the entire top row
    gs = gridspec.GridSpec(2, 2, height_ratios=[1.5, 1])

    # Graph 1: Fixed 2D World Map (Top)
    ax_map = fig.add_subplot(gs[0, :])
    ax_map.set_title("Warehouse Floor Map", fontsize=14, fontweight='bold')
    ax_map.grid(True, linestyle='--', color='#d3d3d3')
    ax_map.set_aspect('equal') # Locks aspect ratio to prevent stretching
    ax_map.set_xlim(-200, 200) # Fixed initial boundaries (4m wide)
    ax_map.set_ylim(-100, 300) # Fixed initial boundaries (4m deep)
    
    path_line, = ax_map.plot([], [], 'r-', linewidth=3, label="Driven Path")
    robot_marker, = ax_map.plot([], [], 'o', color='#708090', markersize=14, zorder=5)
    robot_heading_line, = ax_map.plot([], [], 'b-', linewidth=2, zorder=6)
    ax_map.legend(loc="upper left")

    # Graph 2: Polar Radar Sweep (Bottom Left)
    ax_polar = fig.add_subplot(gs[1, 0], projection='polar')
    ax_polar.set_title("Live Polar Radar", fontsize=12, fontweight='bold', pad=15)
    ax_polar.set_thetamin(0)
    ax_polar.set_thetamax(180)
    ax_polar.set_rmax(100)
    
    # Graph 3: Linear Radar Sweep (Bottom Right)
    ax_linear = fig.add_subplot(gs[1, 1])
    ax_linear.set_title("Linear Sensor Profile", fontsize=12, fontweight='bold')
    ax_linear.set_xlim(180, 0)
    ax_linear.set_ylim(0, 200)
    ax_linear.grid(True, linestyle='-', color='#e0e0e0', alpha=0.5)

    canvas = FigureCanvasTkAgg(fig, master=plot_frame)
    canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    window.bind('<Key>', handle_keypress)
    threading.Thread(target=socket_thread, daemon=True).start()
    window.mainloop()

# --- INTERACTION FUNCS ---
def send_quit():
    window.destroy()

def send_cmd(key):
    global cybot_socket
    if cybot_socket:
        try:
            cybot_socket.sendall(key.encode())
            log_terminal(f"> Sent: [{key.upper()}]")
        except Exception as e:
            log_terminal(f"! Error sending: {e}")

def handle_keypress(event):
    if not isinstance(event.widget, tk.Entry):
        key = event.char.lower()
        if key in ['w', 'a', 's', 'd', 'm', 'x', 'a', 'r', 'g', 'c', 'q','e', ' ']:
            if key == ' ': key = 'x'
            send_cmd(key)

def check_map_expansion(x, y):
    # Only expands the map if the robot drives near the edge. Keeps aspect ratio locked!
    xmin, xmax = ax_map.get_xlim()
    ymin, ymax = ax_map.get_ylim()
    expand = False
    
    if x < xmin + 30: xmin -= 100; expand = True
    if x > xmax - 30: xmax += 100; expand = True
    if y < ymin + 30: ymin -= 100; expand = True
    if y > ymax - 30: ymax += 100; expand = True
    
    if expand:
        ax_map.set_xlim(xmin, xmax)
        ax_map.set_ylim(ymin, ymax)

def log_terminal(message):
    telemetry_text.config(state=tk.NORMAL)
    telemetry_text.insert(tk.END, message + "\n")
    telemetry_text.see(tk.END)
    telemetry_text.config(state=tk.DISABLED)

# --- DRAWING FUNCS ---
def update_telemetry_ui(x, y, heading, b_l, b_r, c_l, c_fl, c_fr, c_r):
    global robot_x, robot_y, robot_heading
    robot_x, robot_y, robot_heading = x, y, heading

    stat_pos.set(f"Robot X: {x:.1f} cm | Y: {y:.1f} cm")
    stat_head.set(f"Robot Angle: {heading}°")
    stat_cliff_l.set(f"Cliff Left: {c_l} | Front Left: {c_fl}")
    stat_cliff_r.set(f"Cliff Right: {c_r} | Front Right: {c_fr}")
    stat_bump.set(f"Bump Left: {b_l} | Right: {b_r}")

    path_x.append(x)
    path_y.append(y)
    path_line.set_data(path_x, path_y)
    robot_marker.set_data([x], [y])
    
    head_rad = math.radians(heading)
    robot_heading_line.set_data([x, x + 15 * math.cos(head_rad)], [y, y + 15 * math.sin(head_rad)])

    check_map_expansion(x, y) # Replaces relim() to stop stretching
    if canvas: canvas.draw_idle()

def update_radar_graphs(angle, ir_raw, ping_dist):
    angles_deg.append(angle)
    ir_raw_vals.append(ir_raw)
    ping_distances.append(ping_dist)

    # 1. Update Linear Graph
    ax_linear.clear()
    ax_linear.set_title("Linear Sensor Profile (Ping vs Raw IR)", fontsize=12, fontweight='bold')
    ax_linear.set_xlim(180, 0) 
    ax_linear.set_ylim(0, 200)
    
    # Scale IR Raw down visually so it fits on the 0-200 chart
    scaled_ir = [val / 20.0 for val in ir_raw_vals]
    
    ax_linear.plot(angles_deg, scaled_ir, color='#e6c200', linewidth=1.5, label='IR Raw (Scaled / 20)')
    ax_linear.plot(angles_deg, ping_distances, color='#d32f2f', linewidth=1.5, marker='.', markersize=4, label='Ping (cm)')
    ax_linear.fill_between(angles_deg, scaled_ir, color='#e6c200', alpha=0.2)
    ax_linear.fill_between(angles_deg, ping_distances, color='#d32f2f', alpha=0.3)
    ax_linear.legend(loc="upper right", fontsize=8)
    
    # 2. Update Polar Graph
    ax_polar.clear()
    ax_polar.set_title("Live Polar Radar", fontsize=12, fontweight='bold', pad=15)
    ax_polar.set_thetamin(0)
    ax_polar.set_thetamax(180)
    theta_rad = [math.radians(a) for a in angles_deg]
    ax_polar.plot(theta_rad, ping_distances, color='#d32f2f', linewidth=2, marker='.', markersize=4)
    ax_polar.fill_between(theta_rad, 0, ping_distances, color='#d32f2f', alpha=0.2)

    # 3. Draw Highlights from detected objects
    for obj in current_scan_objects:
        s_ang, e_ang = obj['start'], obj['end']
        color = 'cyan' if obj['width'] <= 12.0 else 'orange'
        
        # Linear highlight
        ax_linear.axvspan(e_ang, s_ang, color=color, alpha=0.4, zorder=0)
        
        # Polar highlight (thick arcs)
        highlight_theta = np.linspace(math.radians(s_ang), math.radians(e_ang), 10)
        highlight_r = np.full_like(highlight_theta, obj['dist'])
        ax_polar.plot(highlight_theta, highlight_r, color=color, linewidth=10, alpha=0.6, solid_capstyle='round')

    ax_polar.set_rmax(100)
    if canvas: canvas.draw_idle()

def add_map_object(obj_id, s_angle, e_angle, c_angle, dist, width):
    # Store for the Radar highlights
    current_scan_objects.append({'start': s_angle, 'end': e_angle, 'dist': dist, 'width': width})

    # Plot on 2D Map
    abs_angle_rad = math.radians(robot_heading + (c_angle - 90))
    obj_x = robot_x + (dist * math.cos(abs_angle_rad))
    obj_y = robot_y + (dist * math.sin(abs_angle_rad))

    if obj_id >= 900:
        color, label_txt, size = 'red', f"HAZARD {obj_id-900}", 150
    elif width <= 10.0:
        color, label_txt, size = '#00aeff', f"Pillar {obj_id}", max(50, width * 10)
    else:
        color, label_txt, size = '#ff8c00', f"Rack {obj_id}", max(100, width * 10)

    ax_map.scatter(obj_x, obj_y, c=color, s=size, edgecolors='black', zorder=4)
    ax_map.text(obj_x, obj_y + (width/2) + 2, label_txt, color='black', fontsize=8, ha='center', va='bottom', fontweight='bold', zorder=5)
    
    check_map_expansion(obj_x, obj_y)
    
    # Trigger a redraw of the radar to show the new highlight immediately
    if len(angles_deg) > 0:
        update_radar_graphs(angles_deg[-1], ir_raw_vals[-1], ping_distances[-1])

def clear_gui_map():
    global robot_x, robot_y, robot_heading, path_x, path_y
    robot_x, robot_y, robot_heading = 0.0, 0.0, 90
    path_x, path_y = [0.0], [0.0]

    path_line.set_data(path_x, path_y)
    robot_marker.set_data([robot_x], [robot_y])
    robot_heading_line.set_data([], [])
    
    # Reset Fixed Bounds
    ax_map.set_xlim(-200, 200)
    ax_map.set_ylim(-100, 300)

    [col.remove() for col in ax_map.collections if col not in [path_line]]
    [txt.remove() for txt in ax_map.texts]

    if canvas: canvas.draw_idle()
    log_terminal("[GUI] Map Memory Cleared.")
    
def detect_objects_with_ir(filepath, ir_threshold=200, lookback=2, min_width=4):
    """ Reads the raw scan file and uses RAW IR jumps to find object edges. """
    angles = []
    ping_dists = []
    irs = []

    if os.path.exists(filepath):
        with open(filepath, 'r') as f:
            next(f, None)  # Skip the header row
            for line in f:
                data = line.split()
                # data[0]=Angle, data[1]=Ping_Dist, data[2]=IR_Raw
                if len(data) >= 3 and data[0].replace('.','',1).isdigit():
                    angles.append(int(data[0]))
                    ping_dists.append(float(data[1])) 
                    irs.append(int(data[2]))          

    objects = []
    in_object = False
    start_idx = 0

    # Lookback window for edge detection
    for i in range(lookback, len(angles)):
        current_ir = irs[i]
        past_ir = irs[i - lookback]
        ir_diff = current_ir - past_ir

        # Rising edge: Object starts
        if not in_object and ir_diff > ir_threshold:
            in_object = True
            start_idx = max(0, i - (lookback // 2))

        # Falling edge: Object ends
        elif in_object and -ir_diff > ir_threshold:
            in_object = False
            end_idx = min(len(angles)-1, i - (lookback // 2))
            radial_width = angles[end_idx] - angles[start_idx]
            
            if radial_width >= min_width:
                obj_dists = ping_dists[start_idx:end_idx+1]
                if obj_dists:
                    objects.append({
                        'start_angle': angles[start_idx],
                        'end_angle': angles[end_idx],
                        'distance': min(obj_dists)
                    })

    # Catch 180 degree edge case
    if in_object:
        end_idx = len(angles) - 1
        radial_width = angles[end_idx] - angles[start_idx]
        if radial_width >= min_width:
            obj_dists = ping_dists[start_idx:end_idx+1]
            if obj_dists:
                objects.append({
                    'start_angle': angles[start_idx],
                    'end_angle': angles[end_idx],
                    'distance': min(obj_dists)
                })

    return angles, ping_dists, objects

def clean_data_action():
    global angles_deg, ping_distances, ir_raw_vals, current_scan_objects
    
    # You can tweak the 200 threshold here to match what you see on the graph!
    angles, distances, objects = detect_objects_with_ir(raw_file_path, ir_threshold=200)
    
    if not angles:
        log_terminal("[GUI]: No raw data to clean yet.")
        return

    log_terminal(f"[GUI]: Found {len(objects)} objects using RAW IR. Writing cleaned data...")
    
    # Clear live plot arrays
    angles_deg.clear()
    ping_distances.clear()
    ir_raw_vals.clear() 
    current_scan_objects.clear()

    with open(clean_file_path, 'w') as f:
        f.write("Angle\tDistance(cm)\n")
        
        for i in range(len(angles)):
            ang = angles[i]
            dist = 100.0 # Push background noise to edge of polar chart
            
            # Check if this angle falls inside any detected object
            for obj in objects:
                if obj['start_angle'] <= ang <= obj['end_angle']:
                    dist = obj['distance']
                    break
                    
            f.write(f"{ang}\t{dist:.2f}\n")
            
            # Repopulate graph arrays
            angles_deg.append(ang)
            ping_distances.append(dist)
            ir_raw_vals.append(0) # Flatline IR dist in clean view
            
    # Add dummy highlights so the objects show up colored on the radar
    for obj in objects:
         current_scan_objects.append({
             'start': obj['start_angle'], 
             'end': obj['end_angle'], 
             'dist': obj['distance'], 
             'width': 10 # Default to cyan highlight
         })

    # Force redraw of the graphs
    if len(angles_deg) > 0:
        update_radar_graphs(angles_deg[-1], 0, ping_distances[-1])
        
    log_terminal("[GUI]: Graph updated with cleaned data.")

# --- NETWORK THREAD ---
def socket_thread():
    global cybot_socket, angles_deg, ir_distances, ping_distances, current_scan_objects

    HOST = "192.168.1.1" 
    PORT = 288

    try:
        cybot_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        cybot_socket.connect((HOST, PORT))
        cybot = cybot_socket.makefile("rbw", buffering=0)
        window.after(0, log_terminal, "[SYSTEM] Connected to CyBot TCP/IP")
    except Exception as e:
        window.after(0, log_terminal, f"[ERROR] Failed to connect: {e}")
        return

    while True:
        try:
            raw_bytes = cybot.readline()
            if not raw_bytes: break
            clean_msg = raw_bytes.decode().strip()
            if not clean_msg: continue

            if clean_msg.startswith("TEL:"):
                data = clean_msg[4:].split(',')
                try:
                    window.after(0, update_telemetry_ui,
                                 float(data[0]), float(data[1]), int(data[2]),
                                 int(data[7]), int(data[8]),
                                 int(data[3]), int(data[5]), int(data[6]), int(data[4]))
                except ValueError: pass

            elif clean_msg.startswith("RAW:"):
                window.after(0, log_terminal, f"Bot: {clean_msg}")
                data = clean_msg[4:].split(',')
                try:
                    # data[0]=angle, data[1]=ir_dist (IGNORED), data[2]=ping_dist, data[3]=ir_raw
                    angle = int(data[0])
                    ping_dist = float(data[2])
                    ir_raw = int(data[3]) 
                    
                    # Clear arrays & text file at the start of a new sweep
                    if angle <= 2:
                        angles_deg.clear()
                        ping_distances.clear()
                        ir_raw_vals.clear()
                        current_scan_objects.clear()
                        # Overwrite file with headers
                        with open(raw_file_path, 'w') as f:
                            f.write("Angle\tPing_Dist\tIR_Raw\n")
                    
                    # Append new data to the file
                    with open(raw_file_path, 'a') as f:
                        f.write(f"{angle}\t{ping_dist:.2f}\t{ir_raw}\n")
                        
                    window.after(0, update_radar_graphs, angle, ir_raw, ping_dist)
                except (ValueError, IndexError): 
                    pass

            elif clean_msg.startswith("OBJ:"):
                window.after(0, log_terminal, f"Bot: {clean_msg}")
                data = clean_msg[4:].split(',')
                try:
                    window.after(0, add_map_object, int(data[0]), int(data[1]), int(data[2]), int(data[3]), float(data[4]), float(data[5]))
                except ValueError: pass
                
            elif clean_msg == "CMD:RESET_MAP":
                window.after(0, clear_gui_map)

            else:
                window.after(0, log_terminal, f"Bot: {clean_msg}")

        except Exception as e:
            window.after(0, log_terminal, f"[ERROR] Connection lost: {e}")
            break

if __name__ == "__main__":
    main()