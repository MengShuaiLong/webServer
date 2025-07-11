import asyncio
import websockets
import json
import rclpy
from rclpy.node import Node
from common_msgs.msg import VehicleInfo

# 模拟车辆数据
cars = [
    {   
        "type": "vehicle_online",
        "speed": 99,
        "gear": ord('D'),
        "mode": 1,
        "version": "1.0.0",
        "imei": "IMEI123456"
    },
    {   
        "type": "vehicle_online",
        "speed": 101,
        "gear": ord('S'),
        "mode": 2,
        "version": "1.1.0",
        "imei": "IMEI654321"
    },
    {   
        "type": "vehicle_online",
        "speed": 109,
        "gear": ord('S'),
        "mode": 2,
        "version": "1.1.0",
        "imei": "IMEI6543219"
    },
    {
        "type": "vehicle_online",
        "speed": 109,
        "gear": ord('S'),
        "mode": 2,
        "version": "1.1.0",
        "imei": "IMEI65432191"
    }
]

class OtaROS2Node(Node):
    def __init__(self):
        super().__init__('ota_ros2_node')
        # 修改发布者消息类型为 String
        self.vehicle_publisher = self.create_publisher(VehicleInfo, '/ota/upgrade/cmd', 10)
        # 修改订阅者消息类型为 String
        self.upgrade_subscriber = self.create_subscription(
            VehicleInfo,  # 使用标准消息类型 String
            '/communication/cloud/online_truck',
            self.upgrade_command_callback,
            10
        )
        self.websocket_connections = []
        # 初始化 received_data 字典，使用默认值
        self.received_data = {}
        # 移除任务创建代码
        # asyncio.create_task(self.send_data_periodically())

    def publish_vehicle_data(self, start_ota):
        """
        发布车辆数据到 ROS 2 话题
        :param car_data: 车辆数据字典
        """
        self.vehicle_publisher.publish(start_ota)

    def add_connection(self, websocket):
        """
        添加 WebSocket 连接到列表
        :param websocket: WebSocket 连接对象
        """
        self.websocket_connections.append(websocket)

    def remove_connection(self, websocket):
        """
        从列表中移除 WebSocket 连接
        :param websocket: WebSocket 连接对象
        """
        if websocket in self.websocket_connections:
            self.websocket_connections.remove(websocket)

    async def upgrade_command_callback(self, msg):
        """
        处理接收到的升级指令
        :param msg: 接收到的 ROS 2 标准消息
        """
        self.received_data.clear() # 清空 received_data 字典
        try:
            # 初始化车辆信息字典
            vehicle_info = {}
            try:
                # 从接收到的消息中提取车辆信息
                vehicle_info["type"] = 'vehicle_online'
                # 使用 round 函数将速度值保留两位小数
                vehicle_info["speed"] = round(float(msg.speed), 2)

                if msg.gear == 1:
                    vehicle_info["gear"] = 'N'
                elif msg.gear == 9:
                    vehicle_info["gear"] = 'R'
                elif msg.gear == 10:
                    vehicle_info["gear"] = 'P'
                elif msg.gear > 1 and msg.gear < 9:
                    vehicle_info["gear"] = str(msg.gear - 1)
                else: 
                    vehicle_info["gear"] = '未知'


                if msg.mode == 1:
                    vehicle_info["mode"] = "自动驾驶"
                elif msg.mode == 2:
                    vehicle_info["mode"] = "遥控驾驶"
                else:
                    vehicle_info["mode"] = "人工驾驶"

                vehicle_info["version"] = str(msg.version)
                vehicle_info["imei"] = str(msg.imei)

            except AttributeError as attr_err:
                # 处理消息对象缺少属性的情况
                self.get_logger().error(f"消息对象缺少必要属性: {attr_err}")
            except (ValueError, TypeError) as conv_err:
                # 处理类型转换失败的情况
                self.get_logger().error(f"类型转换失败: {conv_err}")

            # 更新存储的数据
            self.received_data[vehicle_info['imei']] = vehicle_info

            # 向所有 WebSocket 客户端发送 received_data 中的所有车辆信息
            for conn in self.websocket_connections:
                try:
                    for key, value in self.received_data.items():
                        # 向当前 WebSocket 连接发送每辆车的信息
                        await conn.send(json.dumps(value))
                except Exception as send_error:
                    self.get_logger().error(f"向 WebSocket 客户端发送数据失败: {send_error}")

        except Exception as general_error:
            self.get_logger().error(f"处理升级指令时出现未知错误: {general_error}")

async def send_upgrade_complete(websocket, imei):
    """
    异步函数，用于在 5 秒后发送升级完成的消息
    :param websocket: WebSocket 连接对象
    :param imei: 车辆的 IMEI 号
    """
    await asyncio.sleep(5)
    response = {
        "imei": imei,
        "type": "upgrade_complete",
        "message": f"车辆 {imei} 升级完成"
    }
    print("升级完成")
    await websocket.send(json.dumps(response))

async def handle_connection(websocket, path, ros2_node):
    print("客户端已连接")
    ros2_node.add_connection(websocket)
    try:
        # 接收客户端消息
        async for message in websocket:
            try:
                # 尝试将消息解析为 JSON 格式
                data = json.loads(message)
                print(f"recv client data: {data}")

                if data.get('type') == 'start_upgrade':
                    # 提取关键信息，使用 get 方法避免 KeyError
                    imei = data.get('imei')
                    file_name = data.get('fileName')

                    # 检查关键信息是否存在
                    if not imei or not file_name:
                        ros2_node.get_logger().error("start_upgrade 消息缺少必要字段: imei 或 fileName")
                        continue

                    # 打印调试信息
                    ros2_node.get_logger().info(f"收到 IMEI 为 {imei} 的开始升级消息，文件名为: {file_name}")

                    # 创建并填充 VehicleInfo 消息
                    vehicle_info = VehicleInfo()
                    vehicle_info.imei = imei
                    vehicle_info.file_name = file_name

                    # 发布消息并记录日志
                    try:
                        ros2_node.publish_vehicle_data(vehicle_info)
                        ros2_node.get_logger().info(f"成功发布 IMEI 为 {imei} 的升级消息")
                    except Exception as e:
                        ros2_node.get_logger().error(f"发布 IMEI 为 {imei} 的升级消息失败: {str(e)}")

                elif data.get('type') == 'get_config':
                    imei = data.get('imei')
                    if imei:
                        ros2_node.get_logger().info(f"收到客户端获取配置请求，IMEI: {imei}")
                    else:
                        ros2_node.get_logger().error("get_config 消息缺少 imei 字段")

                # 检查 imei 是否存在，再创建异步任务
                if 'imei' in data:
                    asyncio.create_task(send_upgrade_complete(websocket, data['imei']))

            except json.JSONDecodeError:

                ros2_node.get_logger().error("收到的消息不是有效的 JSON 格式")

    except websockets.exceptions.ConnectionClosedOK:
        pass
    finally:
        print("客户端已断开连接")
        ros2_node.remove_connection(websocket)

async def run_ros2_node(ros2_node):
    """
    异步运行 ROS 2 节点
    :param ros2_node: ROS 2 节点对象
    """
    while rclpy.ok():
        rclpy.spin_once(ros2_node)
        await asyncio.sleep(0.1)

async def run_server(ros2_node):
    """
    异步运行 WebSocket 服务器和 ROS 2 节点
    :param ros2_node: ROS 2 节点对象
    """
    # 启动 WebSocket 服务器
    server = await websockets.serve(
        lambda ws, path: handle_connection(ws, path, ros2_node),
        "0.0.0.0",
        8080
    )
    # 并发运行 ROS 2 节点和等待 WebSocket 服务器关闭
    await asyncio.gather(
        run_ros2_node(ros2_node),
        server.wait_closed()
    )

def main():
    rclpy.init()
    ros2_node = OtaROS2Node()
    try:
        # 使用 asyncio.run 运行异步任务
        asyncio.run(run_server(ros2_node))
    except KeyboardInterrupt:
        pass
    finally:
        # 销毁节点并关闭 ROS 2
        ros2_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()