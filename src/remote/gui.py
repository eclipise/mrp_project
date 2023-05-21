'''
+++++++++++++++++++++++++

        DEPRECATED

+++++++++++++++++++++++++
'''

# contains the GUI code for the remote computer

import PySimpleGUI as sg
import ipaddress

class GUI:
    def __init__(self, polling_rate):
        self.ip = None
        self.port = None
        self.polling_rate = polling_rate

        self.get_ip()
        self.display_main_window()

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

    def display_main_window(self):
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
                send_message = False

        print("exit")
        window.close()

gui = GUI(50)