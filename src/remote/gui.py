# contains the GUI code for the remote computer

import PySimpleGUI as sg
import ipaddress

class GUI:
    def __init__(self):
        self.ip = None
        self.port = None

        get_ip(self)

        print(self.ip, self.port, sep=':')

def check_ip(ip):
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
        if len(host_addr) != 2 or not check_ip(host_addr[0]) or not 65535 >= int(host_addr[1]) >= 1:
            host_addr = sg.popup_get_text("Enter robot IP (X.X.X.X:port)", title="Enter IP", text_color="Red")
            continue

        self.ip = host_addr[0]
        self.port = int(host_addr[1])
        return

def display_main_window():
    pass



layout = [[sg.Push(), sg.B(image_filename="resources/arrow_up.png", key="-f-"), sg.Push()],
          [sg.B(image_filename="resources/arrow_left.png", key="-l-"), sg.Push(), sg.B(image_filename="resources/arrow_right.png", key="-r-")],
          [sg.Push(), sg.B(image_filename="resources/arrow_down.png", key="-b-"), sg.Push()],
          [sg.Text("Movement Speed %"), sg.Slider(range=(0, 100), default_value=100, expand_x=True, orientation="horizontal", enable_events=True, key="-sl-")], 
          [sg.Text("Turn Sharpness %"), sg.Slider(range=(0, 100), default_value=100, expand_x=True, orientation="horizontal", enable_events=True, key="-sl-")], 
          [sg.Push(), sg.B("Disconnect"), sg.Push()]]

window = sg.Window("MRP Controller", layout, size=(350, 600))

while True:
    event, values = window.read()
    if event == sg.WIN_CLOSED:
        break

    if event == "-sl-":
        print(values["-sl-"])
    
    if event == "-f-":
        print("forward")
    
    if event == "-l-":
        print("left")
    
    if event == "-r-":
        print("right")
    
    if event == "-b-":
        print("back")

window.close()