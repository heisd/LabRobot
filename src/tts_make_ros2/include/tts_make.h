#ifndef __TTS_MAKE_H_
#define __TTS_MAKE_H_

#include <time.h>
#include <thread>
#include <vector>
#include <cstdlib>
#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/int32.hpp>

using namespace std;

class TTS : public rclcpp::Node{
public:
    TTS(const std::string &node_name,
         const rclcpp::NodeOptions &options);
    ~TTS();

    /* 登录讯飞引擎, 播报开机问候语, 并开始监听文本话题 */
    /* Login the iFlytek engine, speak the startup greeting and start
       listening on the text topic. */
    void start();

    /* 合成并播放一段文本(供话题回调与开机问候语复用) */
    /* Synthesize and play a piece of text (shared by the topic callback
       and the startup greeting). */
    int speak(const std::string &text);

private:
    string source_path;
    string appid;
    string voice_name;
    string tts_text;
    string tts_topic;       //接收待合成文本的话题名
    string audio_device;    //播放音频使用的命令前缀(含aplay及声卡设备)
    int rdn;
    int volume;
    int pitch;
    int speed;
    int sample_rate;
    char* result;
    const char* params_l;
    const char* params_s;
    const char* params_f;
    const char* params_t;

    bool logged_in;                 //讯飞引擎是否登录成功
    std::string login_params_str;   //登录参数(需在登录期间保持有效)
    std::string session_params_str; //会话参数(每次合成复用)

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr tts_sub;

    string current_time();
    string ws2s(const std::wstring &wstr);
    wstring s2ws(const std::string &str);

    std::string build_session_params();
    void text_callback(const std_msgs::msg::String::SharedPtr msg);

    int text_to_speech(const char* src_text, const char* des_path, const char* params);
};

#endif
