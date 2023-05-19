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

    par1 = -100
    par2 = 100
    par3 = 5000

    client.send_movement(par1, par2, par3)

    client.close()

if __name__ == "__main__":
    main()