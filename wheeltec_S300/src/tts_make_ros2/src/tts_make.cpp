#include "wav_head.h"
#include "tts_make.h"

using namespace std;

wstring TTS::s2ws(const std::string &str)
{
	using convert_typeX = std::codecvt_utf8<wchar_t>;
	std::wstring_convert<convert_typeX, wchar_t> converterX;

	return converterX.from_bytes(str);
}

string TTS::ws2s(const std::wstring &wstr)
{
	using convert_typeX = std::codecvt_utf8<wchar_t>;
	std::wstring_convert<convert_typeX, wchar_t> converterX;

	return converterX.to_bytes(wstr);
}

/* 获取时间 */
string TTS::current_time()
{
	std::string fmt = ".wav";
    static char t_buf[64];
	time_t now_time = time(NULL);
    struct tm* time = localtime(&now_time);
    strftime(t_buf, 64, "%Y-%m-%d_%H:%M:%S", time);
    std::wstring wtxt = s2ws(t_buf);
	std::string txt_uft8 = ws2s(wtxt);
    txt_uft8 += fmt;
    return txt_uft8;
}

/* 文本合成 */
int TTS::text_to_speech(const char* src_text, const char* des_path, const char* params)
{
	int          ret          = -1;
	FILE*        fp           = NULL;
	const char*  sessionID    = NULL;
	unsigned int audio_len    = 0;
	wave_pcm_hdr wav_hdr      = default_wav_hdr;
	int          synth_status = MSP_TTS_FLAG_STILL_HAVE_DATA;

	if (NULL == src_text || NULL == des_path)
	{
		printf("params is error!\n");
		return ret;
	}
	fp = fopen(des_path, "wb");
	if (NULL == fp)
	{
		printf("open %s error.\n", des_path);
		return ret;
	}
	/* 开始合成 */
	sessionID = QTTSSessionBegin(params, &ret);
	if (MSP_SUCCESS != ret)
	{
		printf("QTTSSessionBegin failed, error code: %d.\n", ret);
		fclose(fp);
		return ret;
	}
	ret = QTTSTextPut(sessionID, src_text, (unsigned int)strlen(src_text), NULL);
	if (MSP_SUCCESS != ret)
	{
		printf("QTTSTextPut failed, error code: %d.\n",ret);
		QTTSSessionEnd(sessionID, "TextPutError");
		fclose(fp);
		return ret;
	}
	printf("正在合成 ...\n");
	fwrite(&wav_hdr, sizeof(wav_hdr) ,1, fp); //添加wav音频头，使用采样率为16000
	while (1)
	{
		/* 获取合成音频 */
		const void* data = QTTSAudioGet(sessionID, &audio_len, &synth_status, &ret);
		if (MSP_SUCCESS != ret)
			break;
		if (NULL != data)
		{
			fwrite(data, audio_len, 1, fp);
		    wav_hdr.data_size += audio_len; //计算data_size大小
		}
		if (MSP_TTS_FLAG_DATA_END == synth_status)
			break;
	}
	printf("\n");
	if (MSP_SUCCESS != ret)
	{
		printf("QTTSAudioGet failed, error code: %d.\n",ret);
		QTTSSessionEnd(sessionID, "AudioGetError");
		fclose(fp);
		return ret;
	}
	/* 修正wav文件头数据的大小 */
	wav_hdr.size_8 += wav_hdr.data_size + (sizeof(wav_hdr) - 8);

	/* 将修正过的数据写回文件头部,音频文件为wav格式 */
	fseek(fp, 4, 0);
	fwrite(&wav_hdr.size_8,sizeof(wav_hdr.size_8), 1, fp); //写入size_8的值
	fseek(fp, 40, 0); //将文件指针偏移到存储data_size值的位置
	fwrite(&wav_hdr.data_size,sizeof(wav_hdr.data_size), 1, fp); //写入data_size的值
	fclose(fp);
	fp = NULL;
	/* 合成完毕 */
	ret = QTTSSessionEnd(sessionID, "Normal");
	if (MSP_SUCCESS != ret)
	{
		printf("QTTSSessionEnd failed, error code: %d.\n",ret);
	}

	return ret;
}

/* 构造会话参数(发音人/资源路径/音量音调语速等), 每次合成均可复用 */
std::string TTS::build_session_params()
{
	std::string session_ori_1 = "engine_type = local,voice_name=";
	std::string session_ori_2 = ", text_encoding = UTF8, tts_res_path = fo|";
	std::string session_ori_3 = "/config/bin/msc/res/tts/xiaoyan.jet;fo|";
	std::string session_ori_4 = "/config/bin/msc/res/tts/common.jet, sample_rate = ";
	std::string session_ori_5 = ", volume = ";
	std::string session_ori_6 = ", pitch = ";
	std::string session_ori_7 = ", rdn = ";
	std::string session_ori_8 = ", speed = ";
	std::string session_fin = session_ori_1 + voice_name + session_ori_2 + source_path +
		session_ori_3 + source_path + session_ori_4 + std::to_string(sample_rate) +
		session_ori_5 + std::to_string(volume) + session_ori_6 + std::to_string(pitch) +
		session_ori_7 + std::to_string(rdn) + session_ori_8 + std::to_string(speed);
	return session_fin;
}

/* 合成并播放一段文本 */
int TTS::speak(const std::string &text)
{
	if (text.empty()) return 0;
	if (!logged_in)
	{
		RCLCPP_WARN(this->get_logger(), "TTS engine not logged in, drop text: %s", text.c_str());
		return -1;
	}

	/* 合成到临时wav文件, 再交给aplay播放 */
	std::string wav_file = "/tmp/tts_make.wav";
	RCLCPP_INFO(this->get_logger(), ">>>>>合成文本: %s", text.c_str());
	int ret = text_to_speech(text.c_str(), wav_file.c_str(), session_params_str.c_str());
	if (MSP_SUCCESS != ret)
	{
		RCLCPP_ERROR(this->get_logger(), "text_to_speech failed, error code: %d.", ret);
		return ret;
	}

	std::string play_cmd = audio_device + wav_file;
	int prc = system(play_cmd.c_str());
	(void)prc;
	return ret;
}

/* 待合成文本话题回调: 收到文本即合成并播放 */
void TTS::text_callback(const std_msgs::msg::String::SharedPtr msg)
{
	speak(msg->data);
}

/* 登录引擎, 播报开机问候语, 并开始监听待合成文本话题 */
void TTS::start()
{
	login_params_str   = "appid = " + appid + ", work_dir = .";
	session_params_str = build_session_params();

	/* 用户登录, 整个生命周期保持登录, 析构时再登出 */
	int ret = MSPLogin(NULL, NULL, login_params_str.c_str());
	if (MSP_SUCCESS != ret)
	{
		RCLCPP_ERROR(this->get_logger(), "MSPLogin failed, error code: %d.", ret);
		logged_in = false;
	}
	else
	{
		logged_in = true;
		RCLCPP_INFO(this->get_logger(), "TTS MSPLogin success!");
	}

	/* 创建待合成文本话题订阅者, 任何节点向该话题发布文本即可让小车说话 */
	tts_sub = this->create_subscription<std_msgs::msg::String>(
		tts_topic, 10,
		[this](const std_msgs::msg::String::SharedPtr msg){ this->text_callback(msg); });

	/* 开机问候语(参数tts_text非空时播报一次, 兼容原有行为) */
	if (!tts_text.empty()) speak(tts_text);

	RCLCPP_INFO(this->get_logger(), "tts_node ready, listening on topic: %s", tts_topic.c_str());
}

/* 初始化 */
TTS::TTS(const std::string &node_name,const rclcpp::NodeOptions &options)
: rclcpp::Node(node_name,options), logged_in(false){
	RCLCPP_INFO(this->get_logger(),"%s node init!\n",node_name.c_str());

	this->declare_parameter<int>("rdn",0);
	this->declare_parameter<int>("volume",0);
	this->declare_parameter<int>("pitch",0);
	this->declare_parameter<int>("speed",0);
	this->declare_parameter<int>("sample_rate",0);
	this->declare_parameter<string>("source_path","");
	this->declare_parameter<string>("appid","");
	this->declare_parameter<string>("voice_name","");
	this->declare_parameter<string>("tts_text","");
	this->declare_parameter<string>("tts_topic","tts_text");
	this->declare_parameter<string>("audio_device","aplay -D plughw:CARD=Device,DEV=0 ");

	this->get_parameter("rdn",rdn);
	this->get_parameter("volume",volume);
	this->get_parameter("pitch",pitch);
	this->get_parameter("speed",speed);
	this->get_parameter("sample_rate",sample_rate);
	this->get_parameter<string>("source_path",source_path);
	this->get_parameter<string>("appid",appid);
	this->get_parameter<string>("voice_name",voice_name);
	this->get_parameter<string>("tts_text",tts_text);
	this->get_parameter<string>("tts_topic",tts_topic);
	this->get_parameter<string>("audio_device",audio_device);
}

TTS::~TTS(){
	if (logged_in) MSPLogout(); //退出登录
	RCLCPP_INFO(this->get_logger(),"tts_node over!\n");
}

int main(int argc,char **argv)
{
	rclcpp::init(argc,argv);
	auto tts_make = std::make_shared<TTS>("tts_node",rclcpp::NodeOptions());
	tts_make->start();
	rclcpp::spin(tts_make);
	rclcpp::shutdown();
	return 0;
}
