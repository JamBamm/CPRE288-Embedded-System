import socket
import tkinter as tk
from tkinter import scrolledtext
import threading
import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# --- GLOBAL DATA STORAGE ---
cybot_socket = None
window = None
canvas = None

# Telemetry / Odometry History
robot_x, robot_y, robot_heading = 0.0, 0.0, 90
path_x, path_y = [0.0], [0.0]

# Radar Sweep Data
angles_rad, ir_distances, ping_distances = [], [], []

# World Map Objects
map_objs_x, map_objs_y = [], []

# Matplotlib Line Objects
path_line = None
robot_marker = None
obj_scatter = None
radar_ir_line = None
radar_ping_line = None

# --- GUI LAYOUT & SETUP ---
def main():
    global window, canvas
    window = tk.Tk()
    window.title("CyBot Final Logistics Dashboard")
    window.geometry("1400x900")

    # LEFT PANEL: Controls & Telemetry
    control_frame = tk.Frame(window, padx=15, pady=15, width=400)
    control_frame.pack(side=tk.LEFT, fill=tk.Y)

    # 1. Telemetry Log
    tk.Label(control_frame, text="LIVE SYSTEM TELEMETRY", font=("Arial", 12, "bold")).pack(pady=(0,5))
    global telemetry_text
    telemetry_text = scrolledtext.ScrolledText(control_frame, width=45, height=12, font=("Courier", 10), bg="#1e1e1e", fg="#00ff00")
    telemetry_text.pack(pady=5)
    telemetry_text.insert(tk.END, "Waiting for connection to base station...\n")
    telemetry_text.config(state=tk.DISABLED)

    # 2. Movement Controls
    tk.Label(control_frame, text="MANUAL OVERRIDE (WASD)", font=("Arial", 12, "bold")).pack(pady=(20,5))
    btn_frame1 = tk.Frame(control_frame)
    btn_frame1.pack()
    tk.Button(btn_frame1, text="Forward (W)", width=12, command=lambda: send_cmd('w')).grid(row=0, column=1, pady=4)
    tk.Button(btn_frame1, text="Left (A)", width=12, command=lambda: send_cmd('a')).grid(row=1, column=0, padx=4)
    tk.Button(btn_frame1, text="Reverse (S)", width=12, command=lambda: send_cmd('s')).grid(row=1, column=1)
    tk.Button(btn_frame1, text="Right (D)", width=12, command=lambda: send_cmd('d')).grid(row=1, column=2, padx=4)

    # 3. Mission Modes
    tk.Label(control_frame, text="MISSION MODES", font=("Arial", 12, "bold")).pack(pady=(25,5))
    btn_frame2 = tk.Frame(control_frame)
    btn_frame2.pack(pady=5)
    tk.Button(btn_frame2, text="MANUAL MODE (M)", width=18, height=2, bg="lightblue", command=lambda: send_cmd('m')).grid(row=0, column=0, padx=5)
    tk.Button(btn_frame2, text="AUTO MISSION (A)", width=18, height=2, bg="lightgreen", command=lambda: send_cmd('a')).grid(row=0, column=1, padx=5)

    tk.Button(control_frame, text="EMERGENCY STOP (SPACE)", width=38, height=2, bg="red", fg="white", font=("Arial", 11, "bold"), command=lambda: send_cmd('x')).pack(pady=15)
    tk.Button(btn_frame2, text="RESET / IDLE (R)", width=38, height=2, bg="orange", command=lambda: send_cmd('r')).grid(row=1, column=0, columnspan=2, pady=10)
    # Quit
    tk.Button(control_frame, text="Disconnect & Quit", width=20, command=send_quit).pack(side=tk.BOTTOM, pady=20)

    # RIGHT PANEL: Dual Graphs
    plot_frame = tk.Frame(window)
    plot_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

    fig = plt.figure(figsize=(10, 10))
    fig.patch.set_facecolor('#f0f0f0')

    # Graph 1: 2D World Map (Top)
    global ax_map, path_line, robot_marker, obj_scatter
    ax_map = fig.add_subplot(211)
    ax_map.set_title("Warehouse Floor Map", fontsize=14, fontweight='bold')
    ax_map.set_xlabel("X Position (cm)")
    ax_map.set_ylabel("Y Position (cm)")
    ax_map.grid(True, linestyle='--', alpha=0.7)
    ax_map.set_aspect('equal', 'datalim')

    path_line, = ax_map.plot([], [], 'b-', linewidth=2, label="Driven Path")
    robot_marker, = ax_map.plot([], [], 'ro', markersize=8, label="CyBot Current")
    obj_scatter = ax_map.scatter([], [], c='orange', s=60, marker='s', edgecolors='black', label="Detected Obstacles")
    ax_map.legend(loc="upper left")

    # Graph 2: Polar Radar Sweep (Bottom)
    global ax_radar, radar_ir_line, radar_ping_line
    ax_radar = fig.add_subplot(212, projection='polar')
    ax_radar.set_title("Live Sensor Sweep", fontsize=14, fontweight='bold', y=1.05)
    ax_radar.set_rmax(150) # 150cm max radius
    ax_radar.set_thetamax(180) # Half circle

    radar_ir_line, = ax_radar.plot([], [], color='orange', linewidth=2.0, label="IR Distance")
    radar_ping_line, = ax_radar.plot([], [], color='cyan', linewidth=2.0, marker='.', markersize=6, label="PING Distance")
    ax_radar.legend(loc='upper right')

    canvas = FigureCanvasTkAgg(fig, master=plot_frame)
    canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    # Keyboard bindings
    window.bind('<Key>', handle_keypress)

    # Start networking thread
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
            log_terminal(f"> Sent command: [{key.upper()}]")
        except Exception as e:
            log_terminal(f"! Error sending command: {e}")

def handle_keypress(event):
    key = event.char.lower()
    if key in ['w', 'a', 's', 'd', 'm', 'x', 'a', 'r', ' ']:
        # Map spacebar to 'x' (E-STOP)
        if key == ' ': key = 'x'
        send_cmd(key)

def log_terminal(message):
    telemetry_text.config(state=tk.NORMAL)
    telemetry_text.insert(tk.END, message + "\n")
    telemetry_text.see(tk.END) # Auto-scroll to bottom
    telemetry_text.config(state=tk.DISABLED)

# --- DRAWING FUNCS ---
def update_telemetry_ui(x, y, heading, b_l, b_r, c_l, c_fl, c_fr, c_r):
    global robot_x, robot_y, robot_heading
    robot_x, robot_y, robot_heading = x, y, heading

    path_x.append(x)
    path_y.append(y)

    path_line.set_data(path_x, path_y)
    robot_marker.set_data([x], [y])

    ax_map.relim()
    ax_map.autoscale_view()
    if canvas: canvas.draw_idle()

def update_radar_ui(angle, ir_dist, ping_dist):
    angles_rad.append(math.radians(angle))
    ir_distances.append(ir_dist)
    ping_distances.append(ping_dist)

    radar_ir_line.set_data(angles_rad, ir_distances)
    radar_ping_line.set_data(angles_rad, ping_distances)
    if canvas: canvas.draw_idle()

def add_map_object(angle, dist, width):
    abs_angle_rad = math.radians(robot_heading + (angle - 90))

    obj_x = robot_x + (dist * math.cos(abs_angle_rad))
    obj_y = robot_y + (dist * math.sin(abs_angle_rad))

    map_objs_x.append(obj_x)
    map_objs_y.append(obj_y)

    obj_scatter.set_offsets(np.c_[map_objs_x, map_objs_y])
    if canvas: canvas.draw_idle()
    
def clear_gui_map():
    global robot_x, robot_y, robot_heading, path_x, path_y, map_objs_x, map_objs_y

    # Reset position variables
    robot_x, robot_y, robot_heading = 0.0, 0.0, 90
    
    # Clear arrays (keep the starting 0.0 point for the path)
    path_x, path_y = [0.0], [0.0]
    map_objs_x, map_objs_y = [], []

    # Update visual lines
    path_line.set_data(path_x, path_y)
    robot_marker.set_data([robot_x], [robot_y])
    
    # Clear the scatter plot (obstacles) by giving it an empty 2D numpy array
    obj_scatter.set_offsets(np.empty((0, 2)))

    # Redraw
    ax_map.relim()
    ax_map.autoscale_view()
    if canvas: canvas.draw_idle()
    
    log_terminal("[GUI] Internal Map Memory Cleared.")
# --- NETWORK THREAD ---
def socket_thread():
    global cybot_socket, angles_rad, ir_distances, ping_distances

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

            # --- PACKET ROUTER ---
            if clean_msg.startswith("TEL:"):
                data = clean_msg[4:].split(',')
                try:
                    window.after(0, update_telemetry_ui,
                                 float(data[0]), float(data[1]), int(data[2]),
                                 int(data[7]), int(data[8]),
                                 int(data[3]), int(data[5]), int(data[6]), int(data[4]))
                except ValueError: pass

            elif clean_msg == "CMD:RESET_MAP":
                window.after(0, clear_gui_map)
            elif clean_msg.startswith("RAW:"):
                data = clean_msg[4:].split(',')
                # Clear radar graph when a new sweep starts (Angle 0 or 2)
                if int(data[0]) <= 2:
                    angles_rad.clear()
                    ir_distances.clear()
                    ping_distances.clear()
                try:
                    window.after(0, update_radar_ui, int(data[0]), float(data[1]), float(data[2]))
                except ValueError: pass

            elif clean_msg.startswith("OBJ:"):
                data = clean_msg[4:].split(',')
                try:
                    window.after(0, add_map_object, int(data[3]), float(data[4]), float(data[5]))
                except ValueError: pass

            else:
                # Direct string print to the GUI terminal
                window.after(0, log_terminal, f"Bot: {clean_msg}")

        except Exception as e:
            window.after(0, log_terminal, f"[ERROR] Connection lost: {e}")
            break

if __name__ == "__main__":
    main()