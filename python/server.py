#!/usr/bin/python3

# ===================================== COPYRIGHT ===================================== #
#                                                                                       #
#  IFRA (Intelligent Flexible Robotics and Assembly) Group, CRANFIELD UNIVERSITY        #
#  Created on behalf of the IFRA Group at Cranfield University, United Kingdom          #
#  E-mail: IFRA@cranfield.ac.uk                                                         #
#                                                                                       #
#  Licensed under the Apache-2.0 License.                                               #
#  You may not use this file except in compliance with the License.                     #
#  You may obtain a copy of the License at: http://www.apache.org/licenses/LICENSE-2.0  #
#                                                                                       #
#  Unless required by applicable law or agreed to in writing, software distributed      #
#  under the License is distributed on an "as-is" basis, without warranties or          #
#  conditions of any kind, either express or implied. See the License for the specific  #
#  language governing permissions and limitations under the License.                    #
#                                                                                       #
#  IFRA Group - Cranfield University                                                    #
#  AUTHORS: Mikel Bueno Viso - Mikel.Bueno-Viso@cranfield.ac.uk                         #
#           Fahad Khan       - f.khan@cranfield.ac.uk                                   #
#           Dr. Seemal Asif  - s.asif@cranfield.ac.uk                                   #
#           Prof. Phil Webb  - p.f.webb@cranfield.ac.uk                                 #
#                                                                                       #
#  Date: May, 2024.                                                                     #
#                                                                                       #
# ===================================== COPYRIGHT ===================================== #

# ======= CITE OUR WORK ======= #
# You can cite our work with the following statement:
# IFRA-Cranfield (2024) ROS 2 Robotiq Gripper Driver. URL: https://github.com/IFRA-Cranfield/ros2_RobotiqGripper.

# IMPORT -> Required libraries:
import rclpy
from rclpy.node import Node
from ros2_robotiqgripper.srv import RobotiqGripper
import socket, time, re

# ===== INPUT PARAMETER ===== #
PARAM_IP = "0.0.0.0"
P_CHECK_IP = False

class ipPARAM(Node):
    
    def __init__(self):

        global PARAM_IP
        global P_CHECK_IP
        
        super().__init__('ros2_robotiq_ip_param')
        self.declare_parameter('IPAddress', "None")

        PARAM_IP = self.get_parameter('IPAddress').get_parameter_value().string_value
        
        if (PARAM_IP == "None"):
            self.get_logger().error('IPAddress ROS2 Parameter was not defined for the ros2_robotiq Service Server.')
            exit()

        else:    
            self.get_logger().info('IPAddress ROS2 Parameter received: ' + PARAM_IP)
            self.get_logger().info("ros2_RobotiqGripper_Service Server generated.")
            
        P_CHECK_IP = True

# Create NODE:
class serviceServer(Node):

    def __init__(self, IP):

        # Initialise ROS 2 Service Server:
        super().__init__('ros2_RobotiqGripper_ServiceServer')
        self.SERVICE = self.create_service(RobotiqGripper, "Robotiq_Gripper", self.ExecuteService)

        self.ip = IP

        # 初始化夹爪：检查激活并设置参数
        self.initialize_gripper()

    def initialize_gripper(self):
        # TCP-IP + SOCKET settings:
        HOST = self.ip
        PORT = 63352
        
        # SOCKET COMMUNICATION:
        SCKT = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        SCKT.settimeout(3)  # Timeout of 3 seconds.

        # OPEN socket:
        try:
            SCKT.connect((HOST, PORT))
        except Exception as e:
            self.get_logger().error(f"ERROR: Failed to connect to gripper during initialization: {str(e)}")
            return  # 连接失败，继续但记录错误

        # 检查激活状态
        SCKT.sendall(b'GET ACT\n')
        data_act = SCKT.recv(2**10).decode('utf-8')
        match_act = re.search(r'ACT (\d+)', data_act)
        if match_act and int(match_act.group(1)) != 1:
            self.get_logger().info("Gripper not activated. Activating now.")
            SCKT.sendall(b'SET ACT 1\n')
            ignore = SCKT.recv(2**10)
            time.sleep(1.0)  # 等待激活完成
            # 验证激活
            SCKT.sendall(b'GET ACT\n')
            data_act = SCKT.recv(2**10).decode('utf-8')
            match_act = re.search(r'ACT (\d+)', data_act)
            if match_act and int(match_act.group(1)) == 1:
                self.get_logger().info("Gripper activated successfully.")
            else:
                self.get_logger().error("ERROR: Failed to activate gripper.")

        # 设置力阈值 (FOR = 50, small force)
        SCKT.sendall(b'SET FOR 50\n')
        ignore = SCKT.recv(2**10)
        # 验证
        SCKT.sendall(b'GET FOR\n')
        data_for = SCKT.recv(2**10).decode('utf-8')
        match_for = re.search(r'FOR (\d+)', data_for)
        if match_for and int(match_for.group(1)) == 50:
            self.get_logger().info("Force threshold set to 50 successfully.")
        else:
            self.get_logger().error("ERROR: Failed to set force threshold.")

        # 设置速度阈值 (SPE = 50, safe speed)
        SCKT.sendall(b'SET SPE 50\n')
        ignore = SCKT.recv(2**10)
        # 验证
        SCKT.sendall(b'GET SPE\n')
        data_spe = SCKT.recv(2**10).decode('utf-8')
        match_spe = re.search(r'SPE (\d+)', data_spe)
        if match_spe and int(match_spe.group(1)) == 50:
            self.get_logger().info("Speed threshold set to 50 successfully.")
        else:
            self.get_logger().error("ERROR: Failed to set speed threshold.")

        SCKT.close()  # 关闭初始化连接

    def ExecuteService(self, request, response):

        # INITIALISE RESPONSE:
        response.success = False
        response.value = -1
        response.average = -1.0
        
        # TCP-IP + SOCKET settings:
        HOST = self.ip
        PORT = 63352
        
        # SOCKET COMMUNICATION:
        SCKT = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        SCKT.settimeout(3) # Timeout of 3 seconds.

        # OPEN socket:
        while True:
            try:
                SCKT.connect((HOST, PORT))
                break
            except TimeoutError:
                response.message = "ERROR: TCP-IP socket connection timed out! Please verify IP address and PORT."
                return(response)
            except ConnectionRefusedError:
                response.message = "ERROR: TCP-IP socket connection was refused! Please verify IP address and PORT."
                return(response)

        if request.action == "CLOSE":
            
            # 确保激活（可选，如果已知未激活）
            # SCKT.sendall(b'SET ACT 1\n')
            # ignore = SCKT.recv(2**10)
            # time.sleep(0.5)  # 等待激活
            
            SCKT.sendall(b'SET POS 255\n')
            ignore = SCKT.recv(2**10)
            SCKT.sendall(b'SET GTO 1\n')  # 新增：启动运动
            ignore = SCKT.recv(2**10)
            
            time.sleep(0.1)  # 短暂延迟，确保命令生效
            
            # 定义阈值：电流阈值 (COU > 20 表示 >200 mA)，位置阈值 (POS < 240 表示未完全关闭)
            CURRENT_THRESHOLD = 3  # 对应 30 mA，根据手册示例调整
            POSITION_THRESHOLD = 240  # 接近关闭但未满，表示可能夹物
            
            # 轮询 OBJ、COU 和 POS，直到运动完成 (OBJ != 0) 或超时
            start_time = time.time()
            obj = 0
            cou = 0
            pos = 0
            while time.time() - start_time < 10:  # 最大超时 10 秒
                # 获取 OBJ
                SCKT.sendall(b'GET OBJ\n')
                data_obj = SCKT.recv(2**10).decode('utf-8')
                match_obj = re.search(r'OBJ (\d+)', data_obj)
                self.get_logger().info(f"Debug: Received OBJ data: {data_obj.strip()}")
                if match_obj:
                    obj = int(match_obj.group(1))
                
                # 获取 COU (电流)
                SCKT.sendall(b'GET COU\n')
                data_cou = SCKT.recv(2**10).decode('utf-8')
                match_cou = re.search(r'COU (\d+)', data_cou)
                self.get_logger().info(f"Debug: Received COU data: {data_cou.strip()}, actual current (mA): {int(match_cou.group(1))*10 if match_cou else 'N/A'}")
                if match_cou:
                    cou = int(match_cou.group(1))
                
                # 获取 POS (位置)
                SCKT.sendall(b'GET POS\n')
                data_pos = SCKT.recv(2**10).decode('utf-8')
                match_pos = re.search(r'POS (\d+)', data_pos)
                self.get_logger().info(f"Debug: Received POS data: {data_pos.strip()}")
                if match_pos:
                    pos = int(match_pos.group(1))
                
                # 如果 OBJ != 0，运动完成，跳出循环
                if obj != 0:
                    self.get_logger().info("Debug: Gripper motion completed.")
                    break
                
                time.sleep(0.5)
            
            if obj == 0:
                response.message = "ERROR: Timeout waiting for gripper motion to complete."
                return response
            
            # 计算百分比关闭
            AVERAGE = round((float(pos) / 255.0) * 100.0, 2)
            
            response.success = True
            response.value = pos
            response.average = AVERAGE
            response.message = "CLOSE command successfully sent to Robotiq gripper. After execution, the gripper is -> " + str(AVERAGE) + "% CLOSED."
            
            # 结合检测结果
            actual_current_ma = cou * 10  # 转换为 mA
            if obj == 2 and cou > CURRENT_THRESHOLD and pos < POSITION_THRESHOLD:
                response.message += " 接触检测：通过电流和位置验证，确认夹住了物体（电流: " + str(actual_current_ma) + " mA）。"
            elif obj == 3 or (pos >= POSITION_THRESHOLD and cou <= CURRENT_THRESHOLD):
                response.message += " 接触检测：无物体，夹爪手指自身接触（电流: " + str(actual_current_ma) + " mA）。"
            else:
                response.message += " 接触检测：不确定状态（OBJ: " + str(obj) + "，电流: " + str(actual_current_ma) + " mA，位置: " + str(pos) + "）。建议检查设置。"
            
            return(response)

        elif request.action == "OPEN":
            
            SCKT.sendall(b'SET POS 0\n')
            ignore = SCKT.recv(2**10)
            SCKT.sendall(b'SET GTO 1\n')  # 优化：启动运动
            ignore = SCKT.recv(2**10)
            
            time.sleep(0.1)  # 短暂延迟，确保命令生效
            
            # 轮询 OBJ，直到运动完成
            start_time = time.time()
            obj = 0
            pos = 0
            while time.time() - start_time < 10:
                SCKT.sendall(b'GET OBJ\n')
                data_obj = SCKT.recv(2**10).decode('utf-8')
                match_obj = re.search(r'OBJ (\d+)', data_obj)
                if match_obj:
                    obj = int(match_obj.group(1))
                
                SCKT.sendall(b'GET POS\n')
                data_pos = SCKT.recv(2**10).decode('utf-8')
                match_pos = re.search(r'POS (\d+)', data_pos)
                if match_pos:
                    pos = int(match_pos.group(1))
                
                if obj != 0:
                    break
                
                time.sleep(0.5)
            
            if obj == 0:
                response.message = "ERROR: Timeout waiting for gripper motion to complete."
                return response
            
            AVERAGE = 100.0 - round((float(pos) / 255.0) * 100.0, 2)
            
            response.success = True
            response.value = pos
            response.average = AVERAGE
            response.message = "OPEN command successfully sent to Robotiq gripper. After execution, the gripper is -> " + str(AVERAGE) + "% OPEN."
            return(response)

        elif request.action == "HALF":
            
            SCKT.sendall(b'SET POS 145\n')
            ignore = SCKT.recv(2**10)
            SCKT.sendall(b'SET GTO 1\n')  # 优化：启动运动
            ignore = SCKT.recv(2**10)
            
            time.sleep(0.1)  # 短暂延迟，确保命令生效
            
            # 轮询 OBJ，直到运动完成
            start_time = time.time()
            obj = 0
            pos = 0
            while time.time() - start_time < 10:
                SCKT.sendall(b'GET OBJ\n')
                data_obj = SCKT.recv(2**10).decode('utf-8')
                match_obj = re.search(r'OBJ (\d+)', data_obj)
                if match_obj:
                    obj = int(match_obj.group(1))
                
                SCKT.sendall(b'GET POS\n')
                data_pos = SCKT.recv(2**10).decode('utf-8')
                match_pos = re.search(r'POS (\d+)', data_pos)
                if match_pos:
                    pos = int(match_pos.group(1))
                
                if obj != 0:
                    break
                
                time.sleep(0.5)
            
            if obj == 0:
                response.message = "ERROR: Timeout waiting for gripper motion to complete."
                return response
            
            AVERAGE = round((float(pos) / 255.0) * 100.0, 2)
            
            response.success = True
            response.value = pos
            response.average = AVERAGE
            response.message = "HALF command successfully sent to Robotiq gripper. After execution, the gripper is -> " + str(AVERAGE) + "% CLOSED."
            return(response)
        else:
            response.message = "ERROR: Valid commands are OPEN/CLOSE. Please try again."
            return(response)

# =================== MAIN =================== #
def main(args=None):

    rclpy.init(args=args)

    # Get IP Address:PROGRAM
    global PARAM_IP
    global P_CHECK_IP

    paramNODE = ipPARAM()
    while (P_CHECK_IP == False):
        rclpy.spin_once(paramNODE)
    paramNODE.destroy_node()
    
    # Initialise NODE:
    GripperNode = serviceServer(PARAM_IP)

    # Spin SERVICE:
    rclpy.spin(GripperNode)
    
    GripperNode.destroy_node
    rclpy.shutdown()

if __name__ == '__main__':
    main()


