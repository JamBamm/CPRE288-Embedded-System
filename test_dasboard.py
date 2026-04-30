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

# Radar Sweep Data (Resets every sweep)
angles_rad, ir_distances, ping_distances = [], [], []

# World Map Objects (Persists)
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
    window.title("CyBot Logistics Dashboard")
    window.geometry("1400x900")

    # LEFT PANEL: Controls & Telemetry
    control_frame = tk.Frame(window, padx=10, pady=10, width=400)
    control_frame.pack(side=tk.LEFT, fill=tk.Y)

    # 1. Telemetry Display
    tk.Label(control_frame, text="LIVE TELEMETRY", font=("Arial", 12, "bold")).pack(pady=(0,5))
    global telemetry_text
    telemetry_text = scrolledtext.ScrolledText(control_frame, width=40, height=8, font=("Courier", 10))
    telemetry_text.pack(pady=5)
    telemetry_text.insert(tk.END, "Waiting for connection...\n")
    telemetry_text.config(state=tk.DISABLED)

    # 2. Movement & Modes
    tk.Label(control_frame, text="LOGISTICS CONTROL", font=("Arial", 12, "bold")).pack(pady=(15,5))
    btn_frame1 = tk.Frame(control_frame)
    btn_frame1.pack()
    tk.Button(btn_frame1, text="Forward (w)", width=12, command=lambda: send_cmd('w')).grid(row=0, column=1, pady=2)
    tk.Button(btn_frame1, text="Left (a)", width=12, command=lambda: send_cmd('a')).grid(row=1, column=0, padx=2)
    tk.Button(btn_frame1, text="Reverse (s)", width=12, command=lambda: send_cmd('s')).grid(row=1, column=1)
    tk.Button(btn_frame1, text="Right (d)", width=12, command=lambda: send_cmd('d')).grid(row=1, column=2, padx=2)

    btn_frame2 = tk.Frame(control_frame)
    btn_frame2.pack(pady=10)
    tk.Button(btn_frame2, text="Manual Mode (m)", width=15, bg="lightblue", command=lambda: send_cmd('m')).grid(row=0, column=0, padx=5)
    tk.Button(btn_frame2, text="Auto Mode (a)", width=15, bg="lightgreen", command=lambda: send_cmd('a')).grid(row=0, column=1, padx=5)
    tk.Button(btn_frame2, text="EMERGENCY STOP", width=32, bg="red", fg="white", font=("Arial", 10, "bold"), command=lambda: send_cmd('x')).grid(row=1, column=0, columnspan=2, pady=5)

    # 3. Diagnostic Pad (1-9)
    tk.Label(control_frame, text="DIAGNOSTIC TESTS", font=("Arial", 12, "bold")).pack(pady=(15,5))
    diag_frame = tk.Frame(control_frame)
    diag_frame.pack()
    tests = [
        ('1: IR ADC', '1'), ('2: PING', '2'), ('3: Servo', '3'),
        ('4: Sweep', '4'), ('5: Move', '5'), ('6: Bumps', '6'),
        ('7: Cliffs', '7'), ('8: Tape', '8'), ('9: HALT', '9')
    ]
    for i, (txt, cmd) in enumerate(tests):
        row, col = divmod(i, 3)
        tk.Button(diag_frame, text=txt, width=12, command=lambda c=cmd: send_cmd(c)).grid(row=row, column=col, padx=2, pady=2)

    # Quit
    tk.Button(control_frame, text="Quit GUI", width=15, command=send_quit).pack(side=tk.BOTTOM, pady=20)

    # RIGHT PANEL: Dual Graphs
    plot_frame = tk.Frame(window)
    plot_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

    fig = plt.figure(figsize=(10, 10))

    # Graph 1: 2D World Map (Top)
    global ax_map, path_line, robot_marker, obj_scatter
    ax_map = fig.add_subplot(211)
    ax_map.set_title("2D Warehouse Map", fontsize=14)
    ax_map.set_xlabel("X Position (cm)")
    ax_map.set_ylabel("Y Position (cm)")
    ax_map.grid(True)
    ax_map.set_aspect('equal', 'datalim')

    # Initialize map graphic objects
    path_line, = ax_map.plot([], [], 'r-', linewidth=2, label="Path")
    robot_marker, = ax_map.plot([], [], 'bo', markersize=8, label="Robot")
    obj_scatter = ax_map.scatter([], [], c='orange', s=50, marker='s', label="Obstacles")
    ax_map.legend()

    # Graph 2: Polar Radar Sweep (Bottom)
    global ax_radar, radar_ir_line, radar_ping_line
    ax_radar = fig.add_subplot(212, projection='polar')
    ax_radar.set_title("Live Sensor Sweep", fontsize=14, y=1.05)
    ax_radar.set_rmax(150) # 150cm max radius
    ax_radar.set_thetamax(180) # Only half circle

    # Initialize radar graphic objects
    radar_ir_line, = ax_radar.plot([], [], color='orange', linewidth=2.0, label="IR Dist")
    radar_ping_line, = ax_radar.plot([], [], color='cyan', linewidth=2.0, marker='o', markersize=4, label="PING Dist")
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
            print(f"[GUI]: Sent command '{key}'")
        except Exception as e:
            print(f"[GUI Error]: {e}")

def handle_keypress(event):
    key = event.char.lower()
    if key in ['w', 'a', 's', 'd', 'm', 'x', 'p', ' ']:
        send_cmd(key)

# --- DRAWING FUNCS ---
def update_telemetry_ui(x, y, heading, b_l, b_r, c_l, c_fl, c_fr, c_r):
    global robot_x, robot_y, robot_heading
    robot_x, robot_y, robot_heading = x, y, heading

    # Update text box
    telemetry_text.config(state=tk.NORMAL)
    telemetry_text.delete(1.0, tk.END)
    info = (
        f"Position: X: {x:.1f} cm | Y: {y:.1f} cm\n"
        f"Heading : {heading} deg\n\n"
        f"Bumps   : Left: {b_l} | Right: {b_r}\n"
        f"Cliffs  : L:{c_l} FL:{c_fl} FR:{c_fr} R:{c_r}\n"
    )
    telemetry_text.insert(tk.END, info)
    telemetry_text.config(state=tk.DISABLED)

    # Update map arrays
    path_x.append(x)
    path_y.append(y)

    # Redraw map
    path_line.set_data(path_x, path_y)
    robot_marker.set_data([x], [y])

    # Auto-adjust map boundaries
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
    # Calculate absolute angle in the world
    # Cybot: 0 is Right, 90 is Straight, 180 is Left
    # Relative angle from straight ahead = (angle - 90)
    # Absolute angle = robot_heading + (angle - 90)
    abs_angle_rad = math.radians(robot_heading + (angle - 90))

    obj_x = robot_x + (dist * math.cos(abs_angle_rad))
    obj_y = robot_y + (dist * math.sin(abs_angle_rad))

    map_objs_x.append(obj_x)
    map_objs_y.append(obj_y)

    # Update scatter plot
    obj_scatter.set_offsets(np.c_[map_objs_x, map_objs_y])
    if canvas: canvas.draw_idle()

# --- NETWORK THREAD ---
def socket_thread():
    global cybot_socket, angles_rad, ir_distances, ping_distances

    HOST = "192.168.1.1" # Change to "127.0.0.1" for local mock testing
    PORT = 288

    try:
        cybot_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        cybot_socket.connect((HOST, PORT))
        cybot = cybot_socket.makefile("rbw", buffering=0)
        print("[System]: Connected to CyBot TCP/IP")
    except Exception as e:
        print(f"[System Error]: Failed to connect. {e}")
        return

    while True:
        try:
            raw_bytes = cybot.readline()
            if not raw_bytes: break
            clean_msg = raw_bytes.decode().strip()
            if not clean_msg: continue

            # --- PACKET ROUTER ---
            if clean_msg.startswith("TEL:"):
                # Example: TEL:X,Y,Heading,L_Cliff,R_Cliff,FL_Cliff,FR_Cliff,LBump,RBump
                data = clean_msg[4:].split(',')
                try:
                    window.after(0, update_telemetry_ui,
                                 float(data[0]), float(data[1]), int(data[2]),
                                 int(data[7]), int(data[8]), # Bumps
                                 int(data[3]), int(data[5]), int(data[6]), int(data[4])) # Cliffs
                except ValueError: pass

            elif clean_msg.startswith("RAW:"):
                # Example: RAW:90,25.5,24.0
                data = clean_msg[4:].split(',')
                # If sweeping starts, clear the old radar lines
                if int(data[0]) == 0 or int(data[0]) == 2:
                    angles_rad.clear()
                    ir_distances.clear()
                    ping_distances.clear()
                try:
                    window.after(0, update_radar_ui, int(data[0]), float(data[1]), float(data[2]))
                except ValueError: pass

            elif clean_msg.startswith("OBJ:"):
                # Example: OBJ:id,s_ang,e_ang,c_ang,dist,width
                data = clean_msg[4:].split(',')
                try:
                    # Send center angle and distance to map logic
                    window.after(0, add_map_object, int(data[3]), float(data[4]), float(data[5]))
                except ValueError: pass

            else:
                # Any standard text goes directly to your background terminal
                print(f"[CyBot]: {clean_msg}")

        except Exception as e:
            print(f"[System Error]: {e}")
            break

if __name__ == "__main__":
    main()