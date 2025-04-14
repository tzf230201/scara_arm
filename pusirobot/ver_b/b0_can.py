import can
import struct
import getpass
import time





bus = None  # Global bus variable

def start_can():
    global bus  # Menandakan bahwa kita akan menggunakan variabel bus global
    user = getpass.getuser()
    if user == "peter":
        bus = can.Bus(channel='can0', interface='socketcan')
        print("CAN bus initialized")

def stop_can():
    global bus  # Menggunakan bus global
    if bus is not None:
        bus.shutdown()
        print("CAN bus shutdown")
    else:
        print("CAN bus not initialized")


MAX_FAILED_CNT = 20
RECV_WAIT = 0.5


# CAN-OPEN LIBRARY
SET_1_BYTE = 0x2F
SET_2_BYTE = 0x2B
SET_3_BYTE = 0x27
SET_4_BYTE = 0x23
SET_OK = 0x60
SET_ERROR = 0x80
READ_REQ = 0x40
RECV_1_BYTE = 0x4F
RECV_2_BYTE = 0x4B
RECV_3_BYTE = 0x47
RECV_4_BYTE = 0x43

ID1 = 0x601
ID2 = 0x602
ID3 = 0x603
ID4 = 0x604

response_id_map = {
    ID1: 0x581,
    ID2: 0x582,
    ID3: 0x583,
    ID4: 0x584
}

NO_ERROR = 0x00
MSG_ERROR = 0x01
TIMEOUT_ERROR = 0x02


def send_can_command(command):
    # print(command)
    can_id, can_data = command.split('#')
    can_id = int(can_id, 16)
    can_data = bytes.fromhex(can_data)

    msg = can.Message(arbitration_id=can_id, data=can_data, is_extended_id=False)
    bus.send(msg)
    time.sleep(0.1)


def read_sdo(request_id):
    response_id = response_id_map.get(request_id)
    error_code = NO_ERROR
    can_id = 0
    value = 0
    
    while(can_id != response_id):
        message = bus.recv(RECV_WAIT)  # Wait up to 0.5 seconds for a message
        if message:
            can_id = message.arbitration_id
        else:
            error_code = TIMEOUT_ERROR
            return error_code, value
            
   # print(f"Data CAN Diterima: can id { can_id:03X} data-> {message.data.hex()} (Panjang: {len(message.data)} byte)")
        
    msg = message.data 
    cs = msg[0]
         
    if(cs == SET_ERROR):
        error_code = MSG_ERROR
    else:
        index = (msg[2] << 8) | msg[1]
        sub_index = msg[3]
        value = struct.unpack('<i', msg[4:8])[0]

    return error_code, value

def write_sdo(request_id, cs, index_id, sub_index_id, data):
    # Convert index_id and sub_index_id to bytes
    index_bytes = struct.pack('<H', index_id)
    sub_index_byte = struct.pack('B', sub_index_id)
    
    # Convert data to bytes (assuming 4-byte data for this example)
    data_bytes = struct.pack('<i', data)
    
    # Construct the CAN data payload
    can_data = bytearray()
    can_data.append(cs)                 # Command Specifier
    can_data.extend(index_bytes)        # Index (2 bytes)
    can_data.extend(sub_index_byte)     # Sub-index (1 byte)
    can_data.extend(data_bytes)         # Data (4 bytes)
    
    # Pad the remaining bytes with zeros if necessary to make it 8 bytes
    while len(can_data) < 8:
        can_data.append(0)
    
    # Create and send the CAN message
    msg = can.Message(arbitration_id=request_id, data=can_data, is_extended_id=False)
    bus.send(msg)
    
def set_sdo(request_id, cs, index_id, sub_index_id, data):
    write_sdo(request_id, cs, index_id, sub_index_id, data)
    error_code, ret = read_sdo(request_id)
    
    return error_code

def req_sdo(request_id, index_id, sub_index_id):
    write_sdo(request_id, READ_REQ, index_id, sub_index_id, 0)
    error_code, ret = read_sdo(request_id)
    
    return ret

def set_req_sdo(request_id, cs, index_id, sub_index_id, data):
    error_code = set_sdo(request_id, cs, index_id, sub_index_id, data)
    ret = 0
    if error_code == NO_ERROR:
        ret = req_sdo(request_id, index_id, sub_index_id)

    return error_code, ret

failed_cnt = 0

def ensure_set_req_sdo(request_id, cs, index_id, sub_index_id, data):
    global failed_cnt
    
    error_code, ret = set_req_sdo(request_id, cs, index_id, sub_index_id, data)
    if ( (error_code != NO_ERROR) or (data != ret) ):
        failed_cnt += 1
        
        if (failed_cnt > MAX_FAILED_CNT):
            failed_cnt = 0  # Reset counter after exceeding limit
            return 1
        
        # Call the function recursively
        return ensure_set_req_sdo(request_id, cs, index_id, sub_index_id, data)
    
    failed_cnt = 0  # Reset counter if successful
    
    return 0

def safe_set_sdo(request_id, cs, index_id, sub_index_id, data):
    error_code = ensure_set_req_sdo(request_id, cs, index_id, sub_index_id, data)
    if (error_code != NO_ERROR):
        # shutdown()
        print(f"emergency off, something wrong with can bus communication")
        


    
def req_nmt(request_id):
    response_id = (request_id - 0x600) & 0xFF
    print(f"response_id: {response_id:02X}")
    error_code = NO_ERROR
    can_id = 0x00
    value = 0
    
    while(can_id != response_id):
        message = bus.recv(0.5)  # Wait up to 0.5 seconds for a message
        if message:
            msg = message.data 
            can_id = msg[0] & 0xFF
            print(f"can_id: {can_id:02X}")
            value = msg[0] & 0x00FF
        else:
            error_code = TIMEOUT_ERROR
            return error_code, value
    
    return error_code, value
            
    