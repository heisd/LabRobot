from lebai import LebaiRobot
from rclpy.node import Node
from rclpy.parameter import Parameter
from lebai_driver.motion.tp_trajectory_handler import TPTrajectoryHandler
from lebai_driver.param_utils import get_joint_names
# from urdf_parser_py.urdf import URDF

# 运动服务接口类
class MotionServiceInterface(Node):
    def __init__(self):
        # 初始话ROS2节点
        super().__init__("motion_service")
        # 声明并获取参数
        self.declare_parameter("controller_joint_names", Parameter.Type.STRING_ARRAY)
        # 地址可以自己设置为乐白机器臂的IP为192.168.0.50
        self.declare_parameter("robot_ip_address", "")
        # 获取参数,为啥这里不用统一规范的self.getparameter调用？
        # 进行了封装
        self.joints_name_ = get_joint_names(self, 'controller_joint_names', "robot_description")
        # 检测获取到的参数的合法性
        if not self.joints_name_:
            self.get_logger().error('controller_joint_names is not assigned!')
            raise ValueError("No 'controller_joint_names' parameter.")
        if not self.has_parameter('robot_ip_address'):
            self.get_logger().info('"robot_ip_address" is not assigned.')
            raise ValueError("No 'robot_ip_address' parameter.")
        self.robot_ip_ = self.get_parameter('robot_ip_address').get_parameter_value().string_value
        # lebai SDK封装的机器人对象,通过机器人的IP地址进行连接,下面是这个类的实现
        """
        def __init__(self, ip, sync=True):
            
            # :ip: 机器人设备 IP
            # :sync: 非移动指令自动同步,设置成flase就是禁用自动同步防止进程卡死
            
            self.ip = ip
            # 公共控制器(RobotController)- 用于标准机器人控制
            self.rcc = grpc.insecure_channel(f'{ip}:5181')
            self.rcs = robot_controller_pb2_grpc.RobotControllerStub(self.rcc)
            # 私有控制器(RobotPrivateController)- 用于高级机器人控制
            self.pcc = grpc.insecure_channel(f'{ip}:5182')
            self.pcs = private_controller_pb2_grpc.RobotPrivateControllerStub(self.pcc)
            # https接口服务
            self.http_service = LebaiHttpService(ip)

            self._sync_flag = sync
        """
        self.lebai_robot_ = LebaiRobot(self.robot_ip_, False)
        # 轨迹处理对象
        self.tp_traj_handler_ = TPTrajectoryHandler(self, self.lebai_robot_)
        # self.xxx_ = MinimalActionServer(self)
        # self.joint_trajectory_action_server_ = JointTrajectoryActionServer(self, self.lebai_robot_, self.joints_name_)
        # self.tp_stream_traj_handler_ = TPStreamTrajectoryHandler(self.lebai_robot_, self.joints_name_)
        
    # 设置析构函数,销毁对象tp_traj_handler_
    def __del__(self):
        self.tp_traj_handler_ = None
    #     self.destroy_node()
