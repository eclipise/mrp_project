import PySimpleGUI as sg
import ipaddress
import time
from client import Client
import sys

class GUI:
    def __init__(self):
        self.polling_rate = 200 # 200 ms
        self.start_session()

    def start_session(self):
        self.get_ip()
        self.client = Client(self.host_addr)
        
        check = self.client.check_connection()

        # If the client cannot reach the server, error and prompt for IP again
        while not check[0]:
            print("Unable to reach server:", check[1])
            sg.popup_error("Unable to reach server")
            self.get_ip()

        self.run_main_window()

    def check_ip(self, ip):
        try:
            ipaddress.ip_address(ip)
            return True
        except ValueError:
            return False

    def get_ip(self):
        # Generates a popup prompt for the IP and port
        host_addr = sg.popup_get_text("Enter robot IP (X.X.X.X:port)", title="Enter IP")

        while True:
            # Exits if the user selects cancel or closes the window
            if host_addr is None:
                sys.exit(0)
            
            # Splits into IP and port across ":"
            host_addr = host_addr.split(":")

            # Checks if the IP and port are valid and re-prompts if they are not
            if len(host_addr) != 2 or not self.check_ip(host_addr[0]) or not 65535 >= int(host_addr[1]) >= 1:
                host_addr = sg.popup_get_text("Enter robot IP (X.X.X.X:port)", title="Enter IP", text_color="Red")
                continue

            self.host_addr = f"{host_addr[0]}:{host_addr[1]}"
            return

    def run_main_window(self):
        INITIAL_SPEED = 100 # Percent of maximum speed 
        INITIAL_TURN = 100  # Percent of maximum turn
        
        # Used to track the interval between messages
        last_message_time = time.time()

        # Used to track the interval between status requests
        last_refresh_time = time.time()

        # --- Control Column Elements ---

        forward = sg.RealtimeButton(
            "", 
            image_filename="resources/arrow_up.png", 
            key="-f-"
        )

        left = sg.RealtimeButton(
            "",
            image_filename="resources/arrow_left.png",
            key="-l-"
        )

        right = sg.RealtimeButton(
            "",
            image_filename="resources/arrow_right.png", 
            key="-r-"
        )
        
        backward = sg.RealtimeButton(
            "",
            image_filename="resources/arrow_down.png",
            key="-b-"
        )

        speed = sg.Slider(
            range=(0, 100), 
            default_value=INITIAL_SPEED, 
            expand_x=True,
            orientation="horizontal", 
            enable_events=True,
            key="-speed_sl-"
        )
        
        turn = sg.Slider(
            range=(0, 100),
            default_value=INITIAL_TURN, 
            expand_x=True,
            orientation="horizontal",
            enable_events=True,
            key="-turn_sl-"
        )

        speed_label = sg.Text("Speed")

        turn_label = sg.Text("Turn")
        
        disconnect = sg.B("Disconnect", key="-disc-")

        # -------------------------------

        # Control column layout
        control_column = [[sg.Push(), forward, sg.Push()],
                          [left, sg.Push(), right],
                          [sg.Push(), backward, sg.Push()],
                          [speed_label, speed], 
                          [turn_label, turn], 
                          [sg.Push(), disconnect, sg.Push()]]

        # --- Display Column Elements ---

        battery_label = sg.Text("Battery")

        battery_value = sg.Text("100", key="-bat-", size=(3, None))

        # -------------------------------

        # Display column layout
        display_column = [[battery_label, battery_value]]

        # Overall UI layout
        layout = [
            [
                sg.Column(control_column, expand_x=True, expand_y=True),
                sg.VSeparator(),
                sg.Column(display_column, vertical_alignment="top")
            ]
        ]

        window = sg.Window(f"MRP Controller ({self.host_addr})",
                           layout,
                           size=(450, 500),
                           finalize=True,
                           use_default_focus=False)

        # Allows window to capture WASD keystrokes
        window.bind("<KeyPress-w>", "+w+")
        window.bind("<KeyRelease-w>", "-w-")
        window.bind("<KeyPress-a>", "+a+")
        window.bind("<KeyRelease-a>", "-a-")
        window.bind("<KeyPress-s>", "+s+")
        window.bind("<KeyRelease-s>", "-s-")
        window.bind("<KeyPress-d>", "+d+")
        window.bind("<KeyRelease-d>", "-d-")

        # Variables for tracking held keys
        w_held = False
        a_held = False
        d_held = False
        s_held = False

        # Controlled by slider on UI
        speed = INITIAL_SPEED
        turn = INITIAL_TURN

        # Used to track whether a message is new and should be sent
        send_message = False 

        while True:  
            # Reads all the events in the window (also necessary to spawn the window).
            # Timeout value is used to refresh the data coming back from the robot.
            event, values = window.read()

            # Refreshes the status if it has been at least one second since the last refresh
            if time.time() - last_refresh_time >= 1:
                last_refresh_time = time.time()

                battery = self.client.get_status()["battery"]
                window["-bat-"].update(battery)

            # Exits the main loop when the window is closed
            if event in (sg.WIN_CLOSED, "-disc-"):
                break
            
            # Event handlers for keystrokes 
            if event == "+w+":
                w_held = True
            
            if event == "+a+":
                a_held = True
            
            if event == "+d+":
                d_held = True
            
            if event == "+s+":
                s_held = True
            
            if event == "-w-":
                w_held = False
            
            if event == "-a-":
                a_held = False
            
            if event == "-s-":
                s_held = False
            
            if event == "-d-":
                d_held = False
                

            # Event handler for changes to the movement slider
            if event == "-speed_sl-":
                speed = int(values["-speed_sl-"])

            # Event handler for changes to the turn slider
            if event == "-turn_sl-":
                turn = int(values["-turn_sl-"])
            
            if w_held:
                message = (speed, 0, self.polling_rate)
                send_message = True
            elif a_held:
                message = (0, turn, self.polling_rate)
                send_message = True
            elif d_held:
                message = (0, -turn, self.polling_rate)
                send_message = True
            elif s_held:
                message = (-speed, 0, self.polling_rate)
                send_message = True
            else:
                # Event handler for the up arrow button
                if event == "-f-":
                    message = (speed, 0, self.polling_rate)
                    send_message = True
                
                # Event handler for the left arrow button
                if event == "-l-":
                    message = (0, turn, self.polling_rate)
                    send_message = True
                
                # Event handler for the right arrow button
                if event == "-r-":
                    message = (0, -turn, self.polling_rate)
                    send_message = True
                
                # Event handler for the down arrow button
                if event == "-b-":
                    message = (-speed, 0, self.polling_rate)
                    send_message = True

            # If there is a new, unsent message
            if send_message:
                # If the last message was sent long enough ago, send a new one
                if time.time() - last_message_time >= (self.polling_rate / 1000):
                    last_message_time = time.time()
                    
                    response = self.client.send_movement(message)
                    
                    print(response)

                    send_message = False
        
        window.close()

def run_console_app():
    # Accepts the full IP and port of the host from the user
    host_addr = input("Enter the address of the host (X.X.X.X:port): ")
    
    client = Client(host_addr)
    
    # Runs a console session to get the three parameters and pass them to the server,
    # loops until stopped by user.
    while True:
        speed = int(input("Speed %[-100, 100]: "))
        if not 100 >= speed >= -100:
            print("Error: speed must be between -100 and 100.") 
            continue
        
        turn = int(input("Turn %[-100, 100]: "))
        if not 100 >= turn >= -100:
            print("Error: turn must be between -100 and 100.") 
            continue

        duration = int(input("Duration (ms): "))
        if not duration > 0:
            print("Error: duration must be greater than 0.") 
            continue
        elif duration < 0:
            print("Error: duration must be greater than 0.")
            continue

        res = client.send_movement((speed, turn, duration))
        print(res)

        if input("Continue [y/n]? ") == 'n':
            break

def run_gui_app():
    gui = GUI()

if __name__ == "__main__":
    # run_console_app()
    run_gui_app()
    