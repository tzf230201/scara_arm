import zmq, time

context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.connect("tcp://localhost:5555")
socket.setsockopt_string(zmq.SUBSCRIBE, "")  # subscribe semua pesan

def control_loop():
    print("Looping control task...")

poller = zmq.Poller()
poller.register(socket, zmq.POLLIN)

rate = 0.1  # 10 Hz → 0.1 detik
while True:
    socks = dict(poller.poll(timeout=1))  # cek ada pesan baru (non-blocking)

    if socket in socks and socks[socket] == zmq.POLLIN:
        msg = socket.recv_string()
        print("Received command:", msg)
        if msg == "homing":
            print(">> Running homing")
        elif msg == "stop":
            print(">> Emergency stop!")

    # tetap jalankan loop periodik
    control_loop()
    time.sleep(rate)
