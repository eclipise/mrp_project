# main program running on the remote computer

from client import Client

def main():
    client = Client()
    
    # accepts the full ip and port of the host from the user,
    # splits into IP and port across ':'
    host_addr = input("Enter the address of the host (X.X.X.X:port): ").split(':')
    host_ip = host_addr[0]
    host_port = int(host_addr[1])
    
    client.connect(host_ip, host_port)
    
    # tells server to accept movement
    client.send_control(0)

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

    client.close()

if __name__ == "__main__":
    main()