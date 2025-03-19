import time
import math

import canopen


class Canopen_ku:

    def __init__(self, bustype1, channel1, baud1):
        self.bustype1 = bustype1
        self.channel1 = channel1
        self.baud1 = baud1
        self.network = canopen.Network()
        self.network.connect(bustype=self.bustype1, channel=self.channel1, bitrate=self.baud1)
        # 扫描网络中的节点
        self.network.scanner.search()
        print('Found node: %s!' % self.network.scanner.nodes)
        if len(self.network.scanner.nodes) > 0:
            self.node_id = self.network.scanner.nodes[0]
            self.node = self.network.add_node(self.node_id, 'PMC007C6SEP2-v374.EDS')  # 自动扫描站点
        else:
            raise Exception("No nodes found on the network.")

    def set_rpdo(self, cob_id, data):
        self.network.send_message(cob_id, data)

    def set_current(self, current):
        self.node.sdo[0x600B].raw = current

    def set_sub_div(self, sub_div):
        self.node.sdo[0x600A].raw = sub_div

    def work_mode(self, mode):
        self.node.sdo[0x6005].raw = mode

    def stop_running(self):
        self.node.sdo[0x6020].raw = 0

    def set_pvt_max_command(self, num):
        self.node.sdo[0x6010][0x03].raw = num

    def set_pvt_work_mode(self, mode):
        self.node.sdo[0x6010][0x02].raw = mode

    def set_pvt_start_index(self, start_index):
        self.node.sdo[0x6010][0x05].raw = start_index

    def set_pvt_end_index(self, end_index):
        self.node.sdo[0x6010][0x06].raw = end_index

    def set_position(self, position):
        self.node.sdo[0x600c].raw = position

    def set_pp_start_speed(self, speed):
        self.node.sdo[0x602d][0x03].raw = speed

    def set_pp_stop_speed(self, speed):
        self.node.sdo[0x602d][0x04].raw = speed

    def set_pp_acce_speed(self, a_speed):
        self.node.sdo[0x602d][0x01].raw = a_speed

    def set_pp_dece_speed(self, d_speed):
        self.node.sdo[0x602d][0x02].raw = d_speed

    def clear_status(self):
        self.node.sdo[0x6001].raw = 7

    def read_status(self):
        value = self.node.sdo[0x6001].raw
        return value

    def read_positon(self):
        value = self.node.sdo[0x600c].raw
        return value


def write_pvt(position, velocity, time_interval):
    # 假设这是发送PVT数据到设备的函数
    # 实际实现需要根据具体设备文档调整
    hex_array = []
    sum1 = (velocity << 32) + position
    for i in range(8):
        # 提取每8位并转换为16进制字符串，格式化为两位
        byte = (sum1 >> (i * 8)) & 0xFF
        hex_str = f"{byte:02X}"
        # print(type(hex_str))
        hex_array.append(hex_str)
    int_list = [int(item, 16) for item in hex_array]
    can_bus.set_rpdo(0x307, int_list)

    hex_array2 = []
    sum2 = (2 << 32) + time_interval
    for i in range(5):
        # 提取每8位并转换为16进制字符串，格式化为两位
        byte2 = (sum2 >> (i * 8)) & 0xFF
        hex_str2 = f"{byte2:02X}"
        # print(type(hex_str))
        hex_array2.append(hex_str2)
    int_list2 = [int(item2, 16) for item2 in hex_array2]
    can_bus.set_rpdo(0x407, int_list2)


def set_pvt1start(start_idx):
    # 设置PVT段的起始索引
    can_bus.set_pvt_start_index(start_idx)


def set_pvt1end(end_idx):
    # 设置PVT段的结束索引
    can_bus.set_pvt_end_index(end_idx)


def start_pvt_write(group_id, id):
    can_bus.set_rpdo(0x000, [group_id, id])


def start_pvt_step():
    can_bus.set_rpdo(0x000, [0x0b, 0x00])


def get_busy_state():
    status = can_bus.read_status()
    return status


can_bus = Canopen_ku('canalystii', 0, 125000)
# 参数定义
time_interval = 100  # 时间间隔(ms)
start_v = 600  # 初始速度
accel = 16000  # 加速度
max_v = 32000  # 最大速度
decel = 16000  # 减速度
end_v = 600  # 结束速度
node_id = 7  # 节点ID
group_id = 0  # 组ID
pt_idx = 1  # 点索引
curr_position = 0
curr_velocity = 0
start_pvt_write(1, node_id)
can_bus.set_sub_div(16)
can_bus.set_current(1600)
can_bus.work_mode(2)
can_bus.stop_running()
can_bus.set_pvt_max_command(400)
can_bus.set_pvt_work_mode(0)
can_bus.set_position(0)
# can_bus.set_rpdo(0x207, [0, 0, 40, 6, 16, 0])
# 文件写入
f = open("pvt_data.csv", "w")

# 加速阶段
for i in range(1, 101):  # 从1到100
    curr_time = (i - 1) * time_interval / 1000
    curr_position = start_v * curr_time + accel * curr_time * curr_time / 2
    curr_velocity = start_v + accel * curr_time
    position_wr = math.floor(curr_position + 0.5)
    velocity_wr = math.floor(curr_velocity + 0.5)

    print(f"PVT{i}({position_wr},{velocity_wr},{time_interval})")
    f.write(f"{position_wr},{velocity_wr},{time_interval}\n")
    write_pvt(position_wr, velocity_wr, time_interval)

    time.sleep(0.005)  # 模拟pusi.sleep(5)，单位为秒

    pt_idx += 1
    if curr_velocity >= max_v:
        break

last_position = curr_position

# 匀速阶段
for i in range(1, 31):  # 从1到30
    curr_time = i * time_interval / 1000
    curr_position = last_position + curr_velocity * curr_time
    position_wr = math.floor(curr_position + 0.5)
    velocity_wr = math.floor(curr_velocity + 0.5)

    print(f"PVT{i}({position_wr},{velocity_wr},{time_interval})")
    f.write(f"{position_wr},{velocity_wr},{time_interval}\n")
    write_pvt(position_wr, velocity_wr, time_interval)

    time.sleep(0.01)  # 模拟pusi.sleep(5)，单位为秒

    pt_idx += 1

last_position = curr_position
last_velocity = curr_velocity

# 减速阶段
for i in range(1, 101):  # 从1到100
    curr_time = i * time_interval / 1000
    curr_velocity = last_velocity - decel * curr_time
    curr_position = last_position + last_velocity * curr_time - decel * curr_time * curr_time / 2
    position_wr = math.floor(curr_position + 0.5)
    velocity_wr = math.floor(curr_velocity + 0.5)

    print(f"PVT{i}({position_wr},{velocity_wr},{time_interval})")
    f.write(f"{position_wr},{velocity_wr},{time_interval}\n")
    write_pvt(position_wr, velocity_wr, time_interval)

    time.sleep(0.01)  # 模拟pusi.sleep(5)，单位为秒

    pt_idx += 1
    if curr_velocity <= end_v:
        break

pvt1_start_idx = 0
pvt1_end_idx = pt_idx - 2

# 设置PVT段
set_pvt1start(pvt1_start_idx)
set_pvt1end(pvt1_end_idx)

# 启动步进运动
can_bus.clear_status()
start_pvt_step()

f.close()
print("Process completed.")
