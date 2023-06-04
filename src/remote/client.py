import requests

class Client:
    def __init__(self, host_addr: str) -> None:
        self.host_addr = f"http://{host_addr}"

    # Attempts to connect to the base address
    def check_connection(self) -> tuple[bool, str]:
        try:
            return (requests.get(self.host_addr).ok, "")
        except requests.exceptions.RequestException as e:
            return (False, e)

    def send_movement(self, instruction: tuple[int, int, int]) -> tuple[bool, str]:
        # Pack values into a dictionary to mimic JSON structure
        message = {
            "speed": instruction[0],
            "turn": instruction[1],
            "duration": instruction[2]
        }

        try:
            response = requests.post(f"{self.host_addr}/move", json=message)
            success = response.ok

            if success:
                arduino_response = response.json()["arduino response"]
            else:
                arduino_response = response.json()["errors"]

            return (success, arduino_response)
        except requests.exceptions.RequestException as e:
            return (False, e)
    
    def get_status(self) -> dict:
        try:
            return requests.get(f"{self.host_addr}/status").json()
        except:
            return None

if __name__ == "__main__":
    host_addr = "http://127.0.0.1:5000"
    client = Client(host_addr)

    print(client.check_connection())
    print(client.send_movement((20, 50, 1001)))
    print(client.get_status())
