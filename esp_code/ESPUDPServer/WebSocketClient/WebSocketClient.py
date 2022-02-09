import websocket
import time

def main():
    serverIP = "192.168.0.164";
    ws = websocket.create_connection("ws://" + serverIP)

    while True:
        message = ws.recv()
        print(message)
        time.sleep(0.01)
    return


if __name__ == "__main__":
    main()





