/****************************************************************/
/* Copyright (c) 2023 WHEELTEC Technology, Inc   								*/
/* function:Serial port analysis																*/
/* 功能：串口解析																									*/
/****************************************************************/
#include "wheeltec_mic.h"

/**************************************
Function: Get the serial port return field
功能: 获取串口返回字段
***************************************/
bool Wheeltec_Mic::UnPackMsgPacket(const string &content, MsgPacket &data)
{
	if (content.size() < 7 || ((unsigned char)content.at(0) != FRAME_HEADER))
		return false;

	data.uid  = content[1] & 0xff;
	data.type = content[2] & 0xff;
	data.size = (content[3] & 0xff) | ((content[4] << 8 & 0xff00));
	data.sid  = (content[5] & 0xff) | ((content[6] << 8 & 0xff00));

	switch((MsgType)data.type)
	{
		case MsgType::AIUI_MSG:
		{
			string info = content.substr(7,data.size);
			data.bytes  = info;
			return true;
		}
			break;
		case MsgType::CONTROL:
		{
			string info = content.substr(7,data.size);
			data.bytes  = info;
			return true;
		}
			break;
		default:
        	break;
	}
    return false;
}

/**************************************
Function: Verify serial port data and parse information
功能: 校验串口数据并解析信息
***************************************/
int Wheeltec_Mic::process_data(const unsigned char *buf, int len)
{
	if (buf[2] == 0xff)
	{
		return -1;
	}

	//校验码校对
	int sum = std::accumulate(buf, buf + len - 1, 0);
	if (((~sum + 1) & 0xff) != buf[len - 1]) 
    {
      // cout<<"recv data not crc, drop\n"<<endl;
      return -1;
    }

    if (UnPackMsgPacket(string((char *)buf,len),MsgPkg))
    {
    	Json::Reader reader;
    	if ((MsgType)MsgPkg.type == MsgType::AIUI_MSG)
    	{
    		Json::Value Aiui_Msg;
    		Json::Value value_iwv;

    		if (reader.parse(MsgPkg.bytes, Aiui_Msg)){
    			//串口反馈信息显示
				//cout<< "Aiui_Msg: "<<  Aiui_Msg <<endl;
    			if (Aiui_Msg["type"].asString() == "aiui_event")
    			{
	    			std_msgs::msg::Int8 awake_msg;
	    			awake_msg.data = 1;
	    			awake_flag_pub->publish(awake_msg);
		    		Json::Value content = Aiui_Msg["content"];
						if (content["eventType"].asString() == "4")
							{
								std::string iwv_msg = content["info"].asString();
								if (reader.parse(iwv_msg,value_iwv))
				    		{
				    			angle = value_iwv["ivw"]["angle"].asInt();
				    			std_msgs::msg::UInt32 angle_msg;
				    			angle_msg.data = angle;
				    			angle_pub->publish(angle_msg);
				    			cout << ">>>>>唤醒角度为: " << angle << "°"<< endl;

				    			std_msgs::msg::String msg;
									msg.data = "小车唤醒";
									voice_words_pub->publish(msg);
				    		}
				    	else
								cout << "reader json fail!"<< endl;
								}    				
    			}
    			else
    			{
    				device_message = Aiui_Msg["content"].asString();
    				process_result = true;
    			}
    		}
    	return 1;
    	}
    }
    return -1;
}

/**************************************
Function: Receive and filter data
功能: 过滤数据
***************************************/
int Wheeltec_Mic::uart_analyse(unsigned char buffer)
{
	static int count=0, frame_len=0;
	Receive_Data[count] = buffer;
    if(Receive_Data[0] != FRAME_HEADER || (count == 1 && Receive_Data[1] != USER_ID))  
      count = 0,frame_len = 0;
    else 
      count++;
	if (count == 7){  
      frame_len = (Receive_Data[4]<<8 | Receive_Data[3]) + 7 + 1;
      if (frame_len > 1024) {
          RCLCPP_ERROR(this->get_logger(), "Frame length exceeds buffer size");
          count = frame_len = 0;
          memset(Receive_Data, 0, sizeof(Receive_Data));
          return 0;
      }
	}
	if(count == frame_len && frame_len > 0){
		int ret = process_data(Receive_Data,frame_len);
		count = 0,frame_len = 0;
		memset(Receive_Data, 0, 1024);
		return ret;
	}
	return 0;
}

/**************************************
Function: Receive the information sent by the device
功能: 接收下位机发送的信息
***************************************/
bool Wheeltec_Mic::Get_Serial_Data()
{
  if (!serial_initialized) return false;

  try {
      unsigned char buffer[1] = {0};
      while (MicArr_Serial.available() > 0) {
          if (MicArr_Serial.read(buffer, 1)) {
              if (uart_analyse(buffer[0])) return true;
          }
      }
  } catch (const serial::IOException& e) {
      RCLCPP_ERROR(this->get_logger(), "Serial read error: %s", e.what());
      handle_serial_error();
  }
	return false;
}

/**************************************
Function: Handle serial port errors
功能: 处理串口异常
***************************************/
void Wheeltec_Mic::handle_serial_error()
{
    serial_initialized = false;
    MicArr_Serial.close();

    //立即广播"麦克风离线"(voice_flag=0): dashboard 语音组件卡变红, 其他节点也能感知
    std_msgs::msg::Int8 flag_msg;
    flag_msg.data = 0;
    voice_flag_pub->publish(flag_msg);
    RCLCPP_WARN(this->get_logger(),
        "麦克风串口 %s 读写异常断开, 尝试重连…", usart_port_name.c_str());

    for (int i = 0; i < 3; ++i) {
        try {
            MicArr_Serial.open();
            if (MicArr_Serial.isOpen()) {
                MicArr_Serial.flush();
                serial_initialized = true;
                flag_msg.data = 1;
                voice_flag_pub->publish(flag_msg);
                RCLCPP_INFO(this->get_logger(), "麦克风串口重连成功");
                return;
            }
        } catch (...) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }
    //这里不再是终点: run() 主循环会周期重试并限频报错, 插回 USB 线即可自动恢复
    RCLCPP_ERROR(this->get_logger(),
        "麦克风串口 %s 重连失败, 将持续后台重试 (检查 USB 线/供电/设备号)",
        usart_port_name.c_str());
}

/**************************************
Function: Loop access to the lower computer data and issue topics
功能: 循环获取下位机数据与发布话题
***************************************/
void Wheeltec_Mic::run()
{
  //串口未就绪不再直接退出: 周期重试 + 限频报错(进 /rosout, dashboard 日志面板可见),
  //开机时没插麦克风、或运行中拔掉再插回, 都能自动恢复, 无需重启节点。
  rclcpp::Time last_retry = this->now();

	while(rclcpp::ok()){
    if (!serial_initialized) {
        if ((this->now() - last_retry).seconds() >= 10.0) {
            last_retry = this->now();
            RCLCPP_ERROR(this->get_logger(),
                "麦克风未连接 (串口 %s 打不开), 正在后台重试…"
                " 请检查: USB 线/集线器供电/udev 规则(/dev/wheeltec_mic 是否存在)",
                usart_port_name.c_str());
            initialize_serial();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        rclcpp::spin_some(this->get_node_base_interface());
        continue;
    }
		Get_Serial_Data();
	  rclcpp::spin_some(this->get_node_base_interface());
	}
}

/**************************************
Function: Initialize serial port with retry
功能: 串口初始化
***************************************/
void Wheeltec_Mic::initialize_serial() {
    const int max_retries = 3;
    serial::Timeout timeout = serial::Timeout::simpleTimeout(1000); 
    for (int retry = 0; retry < max_retries; ++retry) {
        try {
            if (MicArr_Serial.isOpen()) MicArr_Serial.close();
            
            MicArr_Serial.setPort(usart_port_name);
            MicArr_Serial.setBaudrate(serial_baud_rate);
            MicArr_Serial.setTimeout(timeout);
            MicArr_Serial.open();
            
            if (MicArr_Serial.isOpen()) {
                MicArr_Serial.flush();
                serial_initialized = true;
                RCLCPP_INFO(get_logger(), "Serial port initialized successfully");
                
                std_msgs::msg::Int8 flag_msg;
                flag_msg.data = 1;
                voice_flag_pub->publish(flag_msg);
								cout << ">>>>>成功打开麦克风设备" << endl;
								cout << ">>>>>以降噪板设置的唤醒词为准[默认:小微小微] " << endl;
                return;
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN(get_logger(), "麦克风串口 %s 第 %d/%d 次打开失败: %s",
                usart_port_name.c_str(), retry + 1, max_retries, e.what());
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    //打开失败: 广播 voice_flag=0 让 dashboard 语音组件卡变红;
    //run() 主循环每 10s 会再调用本函数重试, 报错也随之周期出现在 /rosout。
    std_msgs::msg::Int8 flag_msg;
    flag_msg.data = 0;
    voice_flag_pub->publish(flag_msg);
    RCLCPP_ERROR(this->get_logger(),
        "麦克风串口 %s 连续 %d 次打开失败, 请检查串口线/USB 供电/设备号!",
        usart_port_name.c_str(), max_retries);
}

/**************************************
Function: Constructor, executed only once, for initialization
功能: 构造函数, 用于初始化
***************************************/
Wheeltec_Mic::Wheeltec_Mic(const std::string &node_name)
:rclcpp::Node(node_name),serial_initialized(false){
	memset(&Receive_Data, 0, sizeof(Receive_Data));

	setupMicArrayServices();

	this->declare_parameter<string>("usart_port_name","/dev/ttyCH343USB0");
	this->declare_parameter<int>("serial_baud_rate",115200);

	this->get_parameter("usart_port_name",usart_port_name);
	this->get_parameter("serial_baud_rate",serial_baud_rate);

	/***唤醒标志位话题发布者创建***/
	awake_flag_pub = this->create_publisher<std_msgs::msg::Int8>("awake_flag",10);
	/***麦克风设备串口打开标志位话题发布者创建***/
	voice_flag_pub = this->create_publisher<std_msgs::msg::Int8>("voice_flag",10);
	/***唤醒角度话题发布者创建***/
	angle_pub = this->create_publisher<std_msgs::msg::UInt32>("awake_angle",10);
	/***命令词话题发布者创建***/
	voice_words_pub = this->create_publisher<std_msgs::msg::String>("voice_words",10);

	initialize_serial();

}

Wheeltec_Mic::~Wheeltec_Mic()
{
	RCLCPP_INFO(this->get_logger(),"wheeltec_mic_node over!\n");
	MicArr_Serial.close();
}

int main(int argc,char **argv)
{
	rclcpp::init(argc,argv);
	auto mic = std::make_shared<Wheeltec_Mic>("wheeltec_mic");
  mic->run();
  rclcpp::shutdown();
	return 0;
}