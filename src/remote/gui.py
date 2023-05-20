# contains the GUI code for the remote computer

import PySimpleGUI as sg

connection_layout = [[sg.Text("MRP IP"), sg.In(focus=True, key="ip_input", size=(20,1))], [sg.Button("Connect", key="connect")]]

# sg.Window(title="MRP Controller", layout=connection_layout, element_justification="c").read()

class GUI:
    def __init__(self):
        self.ip = None

        


def get_ip(self):
    self.ip = sg.popup_get_text("MRP IP")
    return self.ip

def display_main_window():


gui = GUI()