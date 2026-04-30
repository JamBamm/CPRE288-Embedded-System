# Author: Phillip Jones
# Date: 10/30/2023
# Description: Client starter code that combines: 1) Simple GUI, 2) creation of a thread for
#              running the Client socket in parallel with the GUI, and 3) Simple recieven of mock sensor 
#              data for a server/cybot.for collecting data from the cybot.

# General Python tutorials (W3schools):  https://www.w3schools.com/python/

# Serial library:  https://pyserial.readthedocs.io/en/latest/shortintro.html 
import serial
import time  # Time library
# Socket library:  https://realpython.com/python-sockets/  
# See: Background, Socket API Overview, and TCP Sockets  
import socket
import tkinter as tk  # Tkinter GUI library
# Thread library: https://www.geeksforgeeks.org/how-to-use-thread-in-tkinter-python/
import threading
import os  # import function for finding absolute path to this python script
import numpy as np  # NEW: For polar math
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import math

angles_rad = []
distances_cm = []
radar_line = None
canvas = None
gui_send_message = ""
Last_command_Label = None
cybot_socket = None
window = None

##### START Define Functions  #########

# Main: Mostly used for setting up, and starting the GUI
def main():
    global window  # Made global so quit function (send_quit) can access
    global radar_line, canvas
    window = tk.Tk()  # Create a Tk GUI Window
    
    window.geometry("1200x820")

    control_frame = tk.Frame(window, padx=10, pady=10)
    control_frame.pack(side=tk.LEFT, fill=tk.Y)

    plot_frame = tk.Frame(window)
    plot_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

    # Last command label
    global Last_command_Label  # Made global so that Client function (socket_thread) can modify
    Last_command_Label = tk.Label(control_frame, text="Last Command Sent: None",
                                  font=("Arial", 12, "bold"))  # Creat a Label
    Last_command_Label.pack(pady=(0, 20))  # Pack the label into the window for display

    # Cybot Scan command Button
    tk.Button(control_frame, text="Press to Scan (m)", width=20, command=lambda: send_cmd('m')).pack(pady=5)

    tk.Button(control_frame, text="Forward (w)", width=15, command=lambda: send_cmd('w')).pack()
    tk.Button(control_frame, text="Left (a)", width=15, command=lambda: send_cmd('a')).pack()
    tk.Button(control_frame, text="Right (d)", width=15, command=lambda: send_cmd('d')).pack()
    tk.Button(control_frame, text="Reverse (s)", width=15, command=lambda: send_cmd('s')).pack()

    tk.Button(control_frame, text="Toggle (t)", width=20, command=lambda: send_cmd('t')).pack(pady=(15, 5))
    tk.Button(control_frame, text="Auto  (g)", width=20, command=lambda: send_cmd('g')).pack(pady=5)
    #tk.Button(control_frame, text="Clean Data ", width=20, command=clean_data).pack(pady=5)
    tk.Button(control_frame, text="Highlight Objects", width=20,  command=highlight_objects).pack(pady=(0, 15))

    # Quit command Button
    tk.Button(control_frame, text="Abort/Stop (q)", width=15, command=lambda: send_cmd('q')).pack(pady=15)
    tk.Button(control_frame, text="Quit GUI", width=15, command = send_quit).pack(side=tk.BOTTOM, pady=20)

    fig, ax = plt.subplots(subplot_kw={'projection': 'polar'}, figsize=(10, 9))

    ax.set_rscale('symlog', linthresh=100.0)
    ax.set_rmax(300)  # Max distance 300cm
    ax.set_rticks([ 20,  40, 60, 80, 100,200,300])
    ax.set_yticklabels(['20','40', '60','80', '100', '2m', '3m'], fontsize=9)
    ax.set_rlabel_position(22.5)
    ax.set_thetamax(180)
    ax.set_xticks(np.arange(0, np.pi + 0.1, np.pi / 4))
    ax.grid(True)
    ax.set_title("Live CyBot Sensor Sweep (cm)", size=14, y=1.0, pad=-24)

    radar_line, = ax.plot([], [], color='r', linewidth=4.0, marker='o', markersize=4)


    canvas = FigureCanvasTkAgg(fig, master=plot_frame)
    canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    # Create a Thread that will run a fuction assocated with a user defined "target" function.
    # In this case, the target function is the Client socket code
    my_thread = threading.Thread(target=socket_thread, daemon=True)  # Creat the thread
    my_thread.start()  # Start the thread

    window.bind('<Key>', handle_keypress)

    # Start event loop so the GUI can detect events such as button clicks, key presses, etc.
    window.mainloop()


# Quit Button action.  Tells the client to send a Quit request to the Cybot, and exit the GUI
def send_quit():
    global gui_send_message  # Command that the GUI has requested be sent to the Cybot
    global window  # Main GUI window

    gui_send_message = "quit\n"  # Update the message for the Client to send
    time.sleep(1)
    window.destroy()  # Exit GUI


# Scan Button action.  Tells the client to send a scan request to the Cybot
def send_scan():
    global gui_send_message  # Command that the GUI has requested sent to the Cybot

    gui_send_message = "m\n"  # Update the message for the Client to send


def handle_keypress(event):
    valid_commands = ['w', 'a', 's', 'd', 't', 'g', 'q', 'm']
    key = event.char.lower()
    if key in valid_commands:
        send_cmd(key)


def send_cmd(key):
    global cybot_socket, Last_command_Label
    # The moment you click a button or press a key, send it directly to the robot!
    if cybot_socket:
        try:
            cybot_socket.sendall(key.encode())
            Last_command_Label.config(text=f"Last Command Sent: {key.upper()}")
            print(f"[GUI]: Sent '{key}'")
        except Exception as e:
            print(f"[GUI Error]: Could not send command. {e}")
    else:
        print("[GUI Error]: Not connected to CyBot yet.")


def clear_radar():
    global angles_rad, distances_cm, radar_line, canvas
    angles_rad.clear()
    distances_cm.clear()
    if radar_line:
        radar_line.set_data([], [])
    if canvas:
        canvas.draw_idle()


def update_radar(angle_deg, dist_cm, draw_now=False):
    global angles_rad, distances_cm, radar_line, canvas
    
    angles_rad.append(angle_deg * (np.pi / 180.0))
    distances_cm.append(dist_cm)
    
    if radar_line:
        radar_line.set_data(angles_rad, distances_cm)
        
    # ONLY draw if the flag is True
    if canvas and draw_now:
        canvas.draw()              
        window.update_idletasks()


# def plot_data(filepath):
# angle_degrees = []
# distance_cm = []

# with open(filepath, 'r') as f:
# for line in f:
# data = line.split()
# # Ignore headers, empty lines, and the "END" flag
# if len(data) >= 2 and data[0].isdigit():
# angle_degrees.append(float(data[0]))
# distance_cm.append(float(data[1]))

# if not angle_degrees:
# print("No valid data to plot!")
# return

# angle_radians = (np.pi/180) * np.array(angle_degrees)

# fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})
# ax.plot(angle_radians, distance_cm, color='r', linewidth=4.0)
# ax.set_xlabel('Distance (cm)', fontsize=14.0)
# ax.set_ylabel('Angle (degrees)', fontsize=14.0)
# ax.xaxis.set_label_coords(0.5, 0.15)

# # Scale set to 100cm (1 meter)
# ax.set_rmax(100)
# ax.set_rticks([20, 40, 60, 80, 100])
# ax.set_rlabel_position(-22.5)
# ax.set_thetamax(180)
# ax.set_xticks(np.arange(0, np.pi+.1, np.pi/4))
# ax.grid(True)

# ax.set_title("Live CyBot Radar Scan", size=14, y=1.0, pad=-24)
# plt.show()
# # -----------------------------
def detect_objects_with_ir(filepath, ir_threshold=200, lookback=2, min_width=4):
    """
    Reads the raw scan file and uses IR jumps with a lookback window to find object edges.
    """
    angles = []
    distances = []
    irs = []

    if os.path.exists(filepath):
        with open(filepath, 'r') as f:
            for line in f:
                data = line.split()
                if len(data) >= 3 and data[0].replace('.','',1).isdigit():
                    angles.append(int(data[0]))
                    distances.append(float(data[1]))
                    irs.append(int(data[2]))

    objects = []
    in_object = False
    start_idx = 0

    # loopbaack like in lab 7  main
    for i in range(lookback, len(angles)):
        current_ir = irs[i]
        past_ir = irs[i - lookback]
        
        ir_diff = current_ir - past_ir

        # Rising edge: Object starts
        if not in_object and ir_diff > ir_threshold:
            in_object = True
            # Shift the start back slightly to account for the lookback window
            start_idx = max(0, i - (lookback // 2))

        # Falling edge: Object ends
        elif in_object and -ir_diff > ir_threshold:
            in_object = False
            end_idx = min(len(angles)-1, i - (lookback // 2))
            
            radial_width = angles[end_idx] - angles[start_idx]
            
            if radial_width >= min_width:
                obj_dists = distances[start_idx:end_idx+1]
                if obj_dists:
                    objects.append({
                        'start_angle': angles[start_idx],
                        'end_angle': angles[end_idx],
                        'distance': min(obj_dists)
                    })

    #  (180 degree edge case)
    if in_object:
        end_idx = len(angles) - 1
        radial_width = angles[end_idx] - angles[start_idx]
        if radial_width >= min_width:
            obj_dists = distances[start_idx:end_idx+1]
            if obj_dists:
                objects.append({
                    'start_angle': angles[start_idx],
                    'end_angle': angles[end_idx],
                    'distance': min(obj_dists)
                })

    return angles, distances, objects
    
object_highlights = [] # Global to keep track of drawn highlights

def clean_data():
    global angles_rad, distances_cm, radar_line, canvas
    
    absolute_path = os.path.dirname(__file__)
    raw_file = os.path.join(absolute_path, 'sensor-scan.txt')
    clean_file = os.path.join(absolute_path, 'cleaned-scan.txt')

    angles, distances, objects = detect_objects_with_ir(raw_file)
    
    if not angles:
        print("[GUI]: No raw data to clean yet.")
        return

    print(f"[GUI]: Found {len(objects)} objects. Writing cleaned data...")
    
    # Clear the live arrays 
    angles_rad.clear()
    distances_cm.clear()

    with open(clean_file, 'w') as f:
        f.write("Angle\tDistance(cm)\n")
        
        for i in range(len(angles)):
            ang = angles[i]
            dist = 300.0 # Push background noise 
            
            # Check if this angle falls inside any detected object
            for obj in objects:
                if obj['start_angle'] <= ang <= obj['end_angle']:
                    dist = obj['distance']
                    break
                    
            f.write(f"{ang}\t{dist:.2f}\n")
            
            # Repopulate the graph arrays with the cleaned data
            angles_rad.append(ang * (np.pi / 180.0))
            distances_cm.append(dist)

    # Immediately redraw 
    if radar_line:
        radar_line.set_data(angles_rad, distances_cm)
    if canvas:
        canvas.draw_idle()
    
    print("[GUI]: Graph updated with cleaned data.")
    

def highlight_objects():
    global canvas, object_highlights, radar_line
    
    absolute_path = os.path.dirname(__file__)
    raw_file = os.path.join(absolute_path, 'sensor-scan.txt')
    
    #Clear old highlights
    for line in object_highlights:
        try:
            line.remove()
        except ValueError:
            pass
    object_highlights.clear()

    #Detect objects using IR
    _, _, objects = detect_objects_with_ir(raw_file)
    
    if not objects:
        print("[GUI]: No objects detected by IR.")
        return

    print(f"[GUI]: Highlighting {len(objects)} objects found via IR edges...")

    # Draw thick arcs safely on the exact same axes as the radar line
    if radar_line:
        ax = radar_line.axes 
        for obj in objects:
            start_rad = obj['start_angle'] * (np.pi / 180.0)
            end_rad = obj['end_angle'] * (np.pi / 180.0)
            dist = obj['distance']
            
            theta_vals = np.linspace(start_rad, end_rad, 20)
            r_vals = np.full_like(theta_vals, dist)
            
            # zorder=1 BEHIND  red line
            line, = ax.plot(theta_vals, r_vals, 
                            color='cyan', linewidth=12, alpha=0.5, 
                            solid_capstyle='round', zorder=1)
            object_highlights.append(line)
        ax.set_rscale('symlog', linthresh=100.0)
        ax.set_rmax(300)
        ax.set_rticks([ 20,  40, 60, 80, 100,200,300])
        ax.set_yticklabels(['20','40', '60','80', '100', '2m', '3m'], fontsize=9)
        clean_data()
    
    if canvas:
        canvas.draw_idle()
def calculate_global_position(robot_x, robot_y, robot_heading, scan_angle, distance):
    # Convert angles to radians
    # Note: Cybot angles usually require subtracting the scan angle from the heading
    absolute_angle_rad = math.radians(robot_heading + scan_angle)

    obj_x = robot_x + (distance * math.cos(absolute_angle_rad))
    obj_y = robot_y + (distance * math.sin(absolute_angle_rad))

    return obj_x, obj_y
# Client socket code (Run by a thread created in main)
def socket_thread():
    # Define Globals
    global cybot_socket
    global Last_command_Label  # GUI label for displaying the last command sent to the Cybot
    global gui_send_message  # Command that the GUI has requested be sent to the Cybot

    # A little python magic to make it more convient for you to adjust where you want the data file to live
    # Link for more info: https://towardsthecloud.com/get-relative-path-python
    absolute_path = os.path.dirname(__file__)  # Absoult path to this python script
    relative_path = "./"  # Path to sensor data file relative to this python script (./ means data file is in the same directory as this python script)
    full_path = os.path.join(absolute_path, relative_path)  # Full path to sensor data file
    filename = 'sensor-scan.txt'  # Name of file you want to store sensor data from your sensor scan command

    # Choose to create either a UART or TCP port socket to communicate with Cybot (Not both!!)
    # UART BEGIN
    # cybot = serial.Serial('COM100', 115200)  # UART (Make sure you are using the correct COM port and Baud rate!!)
    # UART END

    # TCP Socket BEGIN (See Echo Client example): https://realpython.com/python-sockets/#echo-client-and-server
    # HOST = "127.0.0.1"  # The server's hostname or IP address
    # PORT = 65432           # The port used by the server
    HOST = "192.168.1.1"
    PORT = 288

    try:
        cybot_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        #cybot_socket.settimeout(0.05)
        cybot_socket.connect((HOST, PORT))
        cybot = cybot_socket.makefile("rbw", buffering=0)
        print("Connected to CyBot!")
    except Exception as e:
        print(f"Failed to connect: {e}")
        return

    # cybot_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # Create a socket object
    # cybot_socket.connect((HOST, PORT))   # Connect to the socket  (Note: Server must first be running)

    # cybot = cybot_socket.makefile("rbw", buffering=0)  # makefile creates a file object out of a socket:  https://pythontic.com/modules/socket/makefile
    # # TCP Socket END

    # # Send some text: Either 1) Choose "Hello" or 2) have the user enter text to send
    # send_message = "Hello\n"                            # 1) Hard code message to "Hello", or
    # send_message = input("Enter a message:") + '\n'   # 2) Have user enter text
    #gui_send_message = ""  # Initialize GUI command message to wait
    #send_message = ""

    #Ecybot.write(send_message.encode())  # Convert String to bytes (i.e., encode), and send data to the server

    # print("Sent to server: " + send_message)
    
    scan_point_counter = 0
    # Send messges to server until user sends "quit"
    with open(full_path + filename, 'w') as file_object:
        while True:
            try:
                # Read the data coming from the robot
                raw_bytes = cybot.readline()
                if not raw_bytes:
                    print("[System]: CyBot disconnected.")
                    break
                    
                rx_message = raw_bytes.decode()
                clean_msg = rx_message.strip()
                
                # Ignore empty blank lines
                if not clean_msg:
                    continue
                ## Inside the while True loop in socket_thread()
                #clean_msg = rx_message.strip()

                #if clean_msg.startswith("TEL:"):
                    # Parse Telemetry (Odometry & Hardware)
                    data = clean_msg[4:].split(',')
                    robot_x, robot_y, robot_heading = float(data[0]), float(data[1]), int(data[2])
                    # Update your GUI labels with these variables here!

                #elif clean_msg.startswith("RAW:"):
                    # Parse Scan Data (IR and PING)
                    data = clean_msg[4:].split(',')
                    angle, ir_dist, ping_dist = int(data[0]), float(data[1]), float(data[2])
                    # Send to graphing arrays

               # elif clean_msg.startswith("OBJ:"):
                    # Parse calculated objects
                    data = clean_msg[4:].split(',')
                    # Draw circles on the map

                #else:
                    # Treat anything else as a system message
                    print(f"[Cybot]: {clean_msg}")
                # Did the robot just start a scan
                if "--- SCANNING" in clean_msg:
                    print(f"[CyBot Status]: {clean_msg}")
                    window.after(0, clear_radar)
                
                #Is this raw scan data
                elif len(clean_msg.split()) >= 2 and clean_msg.split()[0].isdigit():
                    file_object.write(rx_message) # Save to file
                    file_object.flush()           
                    
                    # Parse the math and update the live plot!
                    data = clean_msg.split()
                    angle = float(data[0])
                    dist = float(data[1])
                    
                    scan_point_counter += 1
                    
                    time_to_draw = (scan_point_counter % 2 == 0)
                    
                    window.after(0, update_radar, angle, dist, time_to_draw)
                    print(f"[CyBot Radar]: {clean_msg}")

                # a normal status update
                elif "END" in clean_msg:
                    scan_point_counter = 0 # Reset for the next scan
                    window.after(0, lambda: (canvas.draw(), window.update_idletasks()) if canvas else None)
                    print("[CyBot Status]: Scan Complete.")

                else:
                    print(f"[CyBot Status]: {clean_msg}")

            except Exception as e:
                print(f"[System Error]: Connection lost. {e}")
                break
                
    print("[System]: Closing connection...")
    if cybot_socket:
        cybot_socket.close()

            # # Create or overwrite existing sensor scan data file
            # file_object = open(full_path + filename,'w') # Open the file: file_object is just a variable for the file "handler" returned by open()

            # while (rx_message.decode() != "END\n"): # Collect sensor data until "END" recieved
            # rx_message = cybot.readline()   # Wait for sensor response, readline expects message to end with "\n"
            # file_object.write(rx_message.decode())  # Write a line of sensor data to the file
            # print(rx_message.decode()) # Convert message from bytes to String (i.e., decode), then print

            # file_object.close() # Important to close file once you are done with it!!
            # print("Scan Complete. Generating Radar Plot...")
            # window.after(0, plot_data, full_path + filename)

        # else:
            # print("Waiting for server reply\n")
            # rx_message = cybot.readline()  # Wait for a message, readline expects message to end with "\n"
            # print(
                # "Got a message from server: " + rx_message.decode() + "\n")  # Convert message from bytes to String (i.e., decode)

        # # Choose either: 1) Idle wait, or 2) Request a periodic status update from the Cybot
        # # 1) Idle wait: for gui_send_message to be updated by the GUI
        # while gui_send_message == "wait\n":
        # time.sleep(.1)  # Sleep for .1 seconds
        # send_message = gui_send_message

        # # 2) Request a periodic Status update from the Cybot:
        # # every .1 seconds if GUI has not requested to send a new command
        # #time.sleep(.1)
        # #if(gui_send_message == "wait\n"):   # GUI has not requested a new command
        # #        send_message = "status\n"   # Request a status update from the Cybot
        # #else:
        # #        send_message = gui_send_message  # GUI has requested a new command

        # gui_send_message = "wait\n"  # Reset gui command message request to wait

        # cybot.write(send_message.encode()) # Convert String to bytes (i.e., encode), and send data to the server
        # try:
            # rx_message = cybot.readline().decode()
            # if rx_message.strip():
                # print(f"[CyBot Status]: {rx_message.strip()}") 
        # except (socket.timeout, TimeoutError):
            # pass 
        # except OSError as e:
            # if "timed out" in str(e).lower():
                # pass
            # else:
                # break

        # time.sleep(0.01)

    # print("Client exiting, and closing file descriptor, and/or network socket\n")
    # time.sleep(2)  # Sleep for 2 seconds
    # cybot.close()  # Close file object associated with the socket or UART
    # cybot_socket.close()  # Close the socket (NOTE: comment out if using UART interface, only use for network socket option)


##### END Define Functions  #########


### Run main ###
if __name__ == "__main__":
    main()