# main program running on the remote computer

import PySimpleGUI as sg
import ipaddress
from client import Client

# creates and GUI and runs communication with a server.
# 
# polling_rate: the duration of a command, which affects how quickly the robot responds  
class GUI:
    def __init__(self, polling_rate):
        self.ip = None
        self.port = None
        self.polling_rate = polling_rate
        self.client = Client()

    def start_remote_session(self):
        self.get_ip()
        self.client.connect(self.ip, self.port)
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

            self.ip = host_addr[0]
            self.port = int(host_addr[1])
            return

    def run_main_window(self):
        INITIAL_SPEED = 100 # percent of maximum speed 
        INITIAL_TURN = 100  # percent of maximum turn
        STOP_MSG = (0, 0, self.polling_rate)

        # UI layout
        layout = [[sg.Push(), sg.RealtimeButton("", image_filename="resources/arrow_up.png", key="-f-"), sg.Push()],
                [sg.RealtimeButton("", image_filename="resources/arrow_left.png", key="-l-"), sg.Push(), sg.RealtimeButton("", image_filename="resources/arrow_right.png", key="-r-")],
                [sg.Push(), sg.RealtimeButton("", image_filename="resources/arrow_down.png", key="-b-"), sg.Push()],
                [sg.Text("Speed"), sg.Slider(range=(0, 100), default_value=INITIAL_SPEED, expand_x=True, orientation="horizontal", enable_events=True, key="-speed_sl-")], 
                [sg.Text("Turn Sharpness"), sg.Slider(range=(0, 100), default_value=INITIAL_TURN, expand_x=True, orientation="horizontal", enable_events=True, key="-turn_sl-")], 
                [sg.Push(), sg.B("Disconnect"), sg.Push()]]

        # defines a window with title, layout, and size
        window = sg.Window(f"MRP Controller ({self.ip}:{self.port})", layout, size=(350, 600))

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
            if event == sg.WIN_CLOSED:
                break
            
            # event handler for changes to the movement slider
            if event == "-speed_sl-":
                speed = int(values["-speed_sl-"])

            # event handler for changes to the turn slider
            if event == "-turn_sl-":
                turn = int(values["-turn_sl-"])
            
            # if the window has not timed out, a button is being held
            if event != sg.TIMEOUT_EVENT:
                # event handler for the up arrow button
                if event == "-f-":
                    message = (speed, 0, self.polling_rate)
                    send_message = True
                
                # event handler for the left arrow button
                if event == "-l-":
                    message = (0, turn, self.polling_rate)
                    send_message = True
                
                # event handler for the right arrow button
                if event == "-r-":
                    message = (0, -turn, self.polling_rate)
                    send_message = True
                
                # event handler for the down arrow button
                if event == "-b-":
                    message = (-speed, 0, self.polling_rate)
                    send_message = True

            # else no button is being held
            elif message != STOP_MSG:
                message = STOP_MSG
                send_message = True

            # send the message to the server if it is not a repeating stop
            if send_message:
                print(message)
                # tells server to accept realtime movement
                self.client.send_control(0)


                send_message = False

        print("exit")
        
        window.close()
        
        # tells the server to terminate
        self.client.send_control(1)
        
        # terminates the client
        self.client.close()

def run_console_app():
    client = Client()
    
    # accepts the full ip and port of the host from the user,
    # splits into IP and port across ':'
    host_addr = input("Enter the address of the host (X.X.X.X:port): ").split(':')
    host_ip = host_addr[0]
    host_port = int(host_addr[1])
    
    client.connect(host_ip, host_port)
    
    # tells server to accept realtime movement
    client.send_control(0)

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
        if duration > 4294967295:
            print("Error: duration must be less than 4,294,967,295.") 
            continue
        elif duration < 0:
            print("Error: duration must be greater than 0.")
            continue

        client.send_movement(speed, turn, duration)

        if input("Continue [y/n]? ") == 'n':
            break

        # tells server to accept movement
        client.send_control(0)
    
    # tells the server to terminate
    client.send_control(1)

    # terminates the client
    client.close()

def run_gui_app():
    polling_rate = 10 # milliseconds
    gui = GUI(polling_rate)
    gui.start_remote_session()

if __name__ == "__main__":
    # run_console_app()
    run_gui_app()
    