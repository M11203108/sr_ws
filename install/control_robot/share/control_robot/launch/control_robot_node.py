import rclpy
from rclpy.node import Node
import serial
import time
import math
import glovar

from std_msgs.msg import String #ros 傳送 type


class read_serial_node(Node):
    S = 300
    Vmax = 100
    a = 50
    dt = 0.1
    N = [0, 0, 0]  # 取樣次數[加速,等速,減速]
    N[0] = int(Vmax / a / dt)
    At = Vmax / a  # 加速時間
    Smin = a * At**2  # 加減速的行走距離
    print("Smin=", Smin)
    N[1] = int(N[0] + (S - Smin) / Vmax / dt)  # 等速的移動距離/等速移動速度
    N[2] = int(N[1] + Vmax / a / dt)
    tiNum = int(N[0] + N[1] + N[2])
    print(N)
    VList = []
    tList = []
    VfbList = []
    stop = False

    def __init__(self,port):
        super().__init__('read_serial') #node 名稱
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.control_callback) #一定要有
        self.timer = self.create_timer(timer_period, self.read_callback)
        self.i = 0
        self.port = port


    def control_callback(self):
          
        self.moter_vmode(self.port,100,0,0)
        print('start')
        time.sleep(3)
        self.moter_vmode(self.port,0,0,0)
        print('finsh')
        st = time.time()
        Tcmd = self.dt * self.i
        if self.i < self.N[0]:
            Vcmd = self.a * (self.dt * self.i)  # 加速時的速度=at
        elif self.i >= self.N[0] and self.i < self.N[1]:
            Vcmd = self.Vmax
        elif self.i >= self.N[1] and self.i < self.N[2]:
            Vcmd = self.Vmax - self.a * (self.dt * (self.i - self.N[1]))  # 降速時的速度=Vt-at
        else:
            Vcmd = 0  # 停止時的速度=0
        self.VList.append(Vcmd)
        self.tList.append(Tcmd)
        self.VfbList.append(glovar.vx_read)
        self.i += 1
        self.moter_vmode(self.port, Vcmd, 0, 0)
        if st - time.time() < self.dt:
            time.sleep(self.dt - (st - time.time()))


    def read_callback(self):
        self.serial_read(self.port)
    


    def serial_read(self, port):
        self.port.flushInput()  # 清空緩衝區
        rec_array = []
        dt = 0.05  # 0.1通訊一次
        time.sleep(1)
        while glovar.read_flat:
            st = time.time()  # 計時開始
            rec_array.clear()
            r = False
            jump = False
            start = time.time()
            while glovar.read_flat:
                rec = self.port.read(1)
                for c in rec:
                    rc = int(c)
                    # print('c=',c)
                    if rc == 125:  # 偵頭0x7B
                        rec_array.append(rc)
                        r = False
                        jump = True
                    if rc == 123:
                        r = True  # 偵尾0x7D
                if r == True:
                    rec_array.append(rc)
                if jump == True:
                    break
                else:
                    if (time.time() - start) > 0.5:  # 如果超過0.5秒都沒收到資料
                        print("通訊異常！！！！！")  # 未來試圖重新啟用
                        # moter_vmode(port,0,0,0)
                        global stop
                        stop = True
            if len(rec_array) == 24:
                # if rec_array[1] == 0: glovar.stop_alarm=True#小車是否可以作動的訊號#1正常0急停#STM32對應參數為"EN"
                # else:glovar.stop_alarm=False
                # print('rec_array[1]=',rec_array[1])#測試中
                # 速度VX,VY,VZ資訊
                if rec_array[2] < 128:
                    glovar.vx_read = (rec_array[2] << 8) + rec_array[
                        3
                    ]  # [33 182]=33*256+182=8630(STM32運動底盤說明書範例P.11)
                else:
                    rec_array[2] = 255 - rec_array[2]
                    rec_array[3] = 255 - rec_array[3]
                    glovar.vx_read = -(
                        (rec_array[2] << 8) + rec_array[3]
                    )  # [33 182]=33*256+182=8630(STM32運動底盤說明書範例P.11)

                if rec_array[4] < 128:
                    glovar.vy_read = (rec_array[4] << 8) + rec_array[5]
                else:
                    rec_array[4] = 255 - rec_array[4]
                    rec_array[5] = 255 - rec_array[5]
                    glovar.vy_read = -((rec_array[4] << 8) + rec_array[5])

                if rec_array[6] < 128:
                    glovar.vz_read = ((rec_array[6] << 8) + rec_array[7]) / 1000
                else:
                    rec_array[6] = 255 - rec_array[6]
                    rec_array[7] = 255 - rec_array[7]
                    glovar.vz_read = (
                        -(((rec_array[6] << 8) + rec_array[7])) / 1000
                    )  # 單位為0.001rad/s(弧度/秒)(STM32運動底盤說明書範例P.11)
                # 加速規AX,AY,AZ資訊
                # todo
                # 電池電量battery power
                if rec_array[20] < 128:
                    glovar.batteryV = (rec_array[20] << 8) + rec_array[21]
                # time.sleep(0.01)
                self.WheelOdometry(dt)
                print(time.time() - st)
                if time.time() - st < dt:
                    time.sleep(dt - (time.time() - st))

                print('V=',glovar.vx_read,glovar.vy_read,glovar.vz_read)
                print('rec_array=',rec_array)
                self.port.flushInput()
        # return rec_array


    def speed(self,v):
        a = [0, 0]
        if v >= 0:
            a[0] = int(v / 256)
            a[1] = int(v % 256)
        else:
            v = -v
            a[0] = 255 - int(v / 256)
            a[1] = 255 - int(v % 256)
        return a


    def moter_vmode(self,robot, Vx, Vy, Vz):
        x, y, z = self.speed(Vx), self.speed(Vy), self.speed(Vz)
        BCC = 0x7B ^ x[0] ^ x[1] ^ y[0] ^ y[1] ^ z[0] ^ z[1]
        robot.write(
            bytearray([0x7B, 0x00, 0x00, x[0], x[1], y[0], y[1], z[0], z[1], BCC, 0x7D])
        )  # cmd = [0x7B, 0x00, Setzero, x[0], x[1], y[0], y[1], z[0], z[1], BCC, 0x7D]


    def WheelOdometry(self, dt):
        theta = glovar.pz_read
        # Vx=(glovar.lastVx+glovar.vx_read)/2
        # Vy=(glovar.lastVy+glovar.vy_read)/2
        # Vz=(glovar.lastVz+glovar.vz_read)/2
        glovar.pz_read += glovar.vz_read * dt
        glovar.px_read += (
            glovar.vx_read * math.cos(theta) - glovar.vy_read * math.sin(theta)
        ) * dt
        glovar.py_read += (
            glovar.vx_read * math.sin(theta) + glovar.vy_read * math.cos(theta)
        ) * dt
        
    
   

def main(args=None):
    rclpy.init(args=args)
    robot = serial.Serial("/dev/ttyACM0", baudrate=115200, timeout=1)
    
    serial_node_publisher = read_serial_node(port = robot)  #class name

    rclpy.spin(serial_node_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    serial_node_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()