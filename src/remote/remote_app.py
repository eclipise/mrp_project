# main program running on the remote computer

import PySimpleGUI as sg
import ipaddress
import time
from client import Client

class GUI:
    def __init__(self):
        self.polling_rate = 200 # 200ms
        self.start_session()

    def start_session(self):
        self.get_ip()
        self.client = Client(self.host_addr)
        self.run_main_window()

    def check_ip(self, ip):
        try:
            ipaddress.ip_address(ip)
            return True
        except ValueError:
            return False

    def get_ip(self):
        # generates a popup prompt for the ip and port
        host_addr = sg.popup_get_text("Enter robot IP (X.X.X.X:port)", title="Enter IP")
            
        while True:
            # returns if the user selects cancel or closes the window
            if host_addr is None:
                return
            
            # splits into ip and port across ":"
            host_addr = host_addr.split(":")

            # checks if the ip and port are valid and re-prompts if they are not
            if len(host_addr) != 2 or not self.check_ip(host_addr[0]) or not 65535 >= int(host_addr[1]) >= 1:
                host_addr = sg.popup_get_text("Enter robot IP (X.X.X.X:port)", title="Enter IP", text_color="Red")
                continue

            self.host_addr = f"{host_addr[0]}:{host_addr[1]}"
            return

    def run_main_window(self):
        INITIAL_SPEED = 100 # percent of maximum speed 
        INITIAL_TURN = 100  # percent of maximum turn
        STOP_MSG = (0, 0, self.polling_rate)
        
        # used to track the interval between messages
        last_message_time = time.time()

        # UI layout
        layout = [[sg.Push(), sg.RealtimeButton("", image_filename="resources/arrow_up.png", key="-f-"), sg.Push()],
                [sg.RealtimeButton("", image_filename="resources/arrow_left.png", key="-l-"), sg.Push(), sg.RealtimeButton("", image_filename="resources/arrow_right.png", key="-r-")],
                [sg.Push(), sg.RealtimeButton("", image_filename="resources/arrow_down.png", key="-b-"), sg.Push()],
                [sg.Text("Speed"), sg.Slider(range=(0, 100), default_value=INITIAL_SPEED, expand_x=True, orientation="horizontal", enable_events=True, key="-speed_sl-")], 
                [sg.Text("Turn Sharpness"), sg.Slider(range=(0, 100), default_value=INITIAL_TURN, expand_x=True, orientation="horizontal", enable_events=True, key="-turn_sl-")], 
                [sg.Push(), sg.B("Disconnect", key="-disc-"), sg.Push()]]

        # defines a window with title, layout, and size
        window = sg.Window(f"MRP Controller ({self.host_addr})", layout, size=(350, 500), finalize=True, use_default_focus=False)

        # allows window to capture WASD keystrokes
        window.bind("<KeyPress-w>", "+w+")
        window.bind("<KeyRelease-w>", "-w-")
        window.bind("<KeyPress-a>", "+a+")
        window.bind("<KeyRelease-a>", "-a-")
        window.bind("<KeyPress-s>", "+s+")
        window.bind("<KeyRelease-s>", "-s-")
        window.bind("<KeyPress-d>", "+d+")
        window.bind("<KeyRelease-d>", "-d-")

        # variables for tracking held keys
        w_held = False
        a_held = False
        d_held = False
        s_held = False

        # controlled by slider on UI
        speed = INITIAL_SPEED
        turn = INITIAL_TURN

        # tuple that is transmitted to the server (speed, turn, duration)
        message = STOP_MSG

        # used to track whether a message is new and should be sent, which right now
        # just prevents the stop message from being sent infinitely
        send_message = False 

        while True:  
            # reads all the events in the window (also necessary to spawn the window).
            # timeout value used to determine when no buttons are being held.
            event, values = window.read(timeout=self.polling_rate)
            
            # exits the main loop when the window is closed
            if event in (sg.WIN_CLOSED, "-disc-"):
                break
            
            # event handlers for keystrokes 
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
                

            # event handler for changes to the movement slider
            if event == "-speed_sl-":
                speed = int(values["-speed_sl-"])

            # event handler for changes to the turn slider
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
            # if the window has not timed out, a GUI button is being held
            elif event != sg.TIMEOUT_EVENT:
                # event handler for the up arrow button and w key
                if event == "-f-":
                    message = (speed, 0, self.polling_rate)
                    send_message = True
                
                # event handler for the left arrow button and a key
                if event == "-l-":
                    message = (0, turn, self.polling_rate)
                    send_message = True
                
                # event handler for the right arrow button and d key
                if event == "-r-":
                    message = (0, -turn, self.polling_rate)
                    send_message = True
                
                # event handler for the down arrow button and s key
                if event == "-b-":
                    message = (-speed, 0, self.polling_rate)
                    send_message = True

            # else no button is being held
            elif message != STOP_MSG:
                message = STOP_MSG
                send_message = True

            # send the message to the server if it is not a repeating stop
            if send_message:
                # print(message)

                # if the last message was sent long enough ago, send a new one
                if time.time() - last_message_time >= (self.polling_rate / 1000):
                    last_message_time = time.time()
                    self.client.send_movement(message)
                    send_message = False
        
        window.close()

def run_console_app():
    # accepts the full ip and port of the host from the user
    host_addr = input("Enter the address of the host (X.X.X.X:port): ")
    
    client = Client(host_addr)
    
    # runs a console session to get the three parameters and pass them to the server,
    # loops until stopped by user
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
    