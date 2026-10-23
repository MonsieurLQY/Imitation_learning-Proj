#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int32_multi_array.hpp" 
#include "std_msgs/msg/int8.hpp"
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>
#include <iomanip>
#include <limits.h>
#include <queue>
#include <array>
#include <atomic>


#include "../include/wiseglove18/Wiseglove.h"

/*
 * WiseGlove 数据发布节点
 * ======================
 *
 * 本文件位于项目数据流的最上游，负责完成：
 *
 *   WiseGlove 硬件
 *       -> 串口 /dev/teleop/glove
 *       -> 本节点 glove_data_pub
 *       -> /quat_data（16 个 float，设计频率约 50 Hz）
 *       -> arm_tele_real（姿态转换与逆运动学）
 *
 * 教学主线只需重点关注下面四处：
 *   1. 构造函数中的 quat_publisher_：创建姿态 Topic 发布者；
 *   2. 20 ms 的 timer_arm_hz：决定约 50 Hz 的读取/发布节奏；
 *   3. timer_callback_arm()：读取并发布四元数；
 *   4. keyboard_input_thread() 的按键 'a'：重置姿态参考并打开/关闭发布。
 *
 * 文件中 angle_data、scaled_data 等 Topic 是手指传感器数据支线，不进入
 * “人体姿态 -> IK -> MPC -> 真机”这条机械臂遥操作主链路。
 */

class GloveDataPub : public rclcpp::Node
{
    
public:
    GloveDataPub() : Node("glove_data_pub")
    {
        // create_publisher<消息类型>(Topic 名, QoS 队列深度)。
        // 这里使用的是相对 Topic 名；节点位于根命名空间时，实际看到的是
        // /angle_data、/scaled_data、/dataOrg_data、/quat_data、/quatOrg_data。
        // 数字 10 是默认 QoS 配置中的历史队列深度。
        angle_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("angle_data", 10);
        scaled_data_publisher_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("scaled_data", 10);
        dataOrg_publisher_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("dataOrg_data", 10);
        // 教学主链路的关键发布者：arm_tele_real 将订阅 /quat_data。
        quat_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("quat_data", 10);
        quatOrg_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("quatOrg_data", 10);
        
        // 外部节点可向 /glove_calib_cmd 发布 Int8，控制手指传感器标定。
        glove_calib_subscriber_ = this->create_subscription<std_msgs::msg::Int8>("glove_calib_cmd",
            10, std::bind(&GloveDataPub::glove_calib_callback, this, std::placeholders::_1)
        );

        // 收到数值 1 时，把当前 IMU 姿态设为新的参考姿态。
        glove_imu_calib_subscriber_ = this->create_subscription<std_msgs::msg::Int8>("glove_imu_calib_cmd",
            10, 
            [this](const std_msgs::msg::Int8::SharedPtr msg)
            {
                if (msg->data == 1)
                {
                    m_glove->ResetQuat();
                }
            }
        );

        // 机械臂姿态定时器：每 20 ms 回调一次，理论频率 1000/20 = 50 Hz。
        // 注意：只有 sent_quat_flag=true 时，回调才真正读取和发布四元数。
        timer_arm_hz = this->create_wall_timer(
            std::chrono::milliseconds(20),
            // std::chrono::microseconds(12500),
            std::bind(&GloveDataPub::timer_callback_arm, this));

        // 手指数据支线也以 20 ms（约 50 Hz）读取并发布。
        timer_hand_hz = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&GloveDataPub::timer_callback_hand, this));
            
        // 键盘监听不能阻塞 ROS 2 的 spin，因此放入独立线程。
        keyboard_thread_ = std::thread(&GloveDataPub::keyboard_input_thread, this);

        // 程序默认使用此设备路径连接手套，波特率在 wg_init() 中设置。
        char port[] = "/dev/teleop/glove";
        if (wg_init(port))
        {
            RCLCPP_INFO(this->get_logger(), "wiseglove init success!");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "wiseglove init failed!");
            rclcpp::shutdown();
        }
 
    }


    ~GloveDataPub()
    {
        // 节点销毁前等待键盘线程退出，防止线程仍访问已经销毁的节点对象。
        if (keyboard_thread_.joinable()) {
            keyboard_thread_.join();
        }
    }


private:

    int wg_init(char* port)
    {
        // Wiseglove 是厂商/项目封装的硬件接口类，不是 ROS 2 类。
        m_glove = new Wiseglove();

        if (m_glove == NULL) return 0;

        // 打开串口并使用 115200 波特率与手套通信。
        bool isopen = m_glove->Open(port, 115200);
        if (!isopen)
        {
            printf("open port error\n");
            return 0;
        }

        // 读取序列号和型号只用于启动日志，失败不会中止节点。
        bool sn_ret = m_glove->GetSn(sn);
        if (sn_ret)
        {
            RCLCPP_INFO(this->get_logger(),"sn:%s\n", sn);
        }
        bool model_ret = m_glove->GetModel(model);
        if (model_ret)
        {
            RCLCPP_INFO(this->get_logger(),"model:%s\n", model);
        }
        return 1;
    }

    unsigned int wg_getdata(unsigned short* data)
    {
        // 这些 wg_get...() 函数是对 Wiseglove API 的薄封装。
        // 返回值是设备时间戳；返回 0 被本文件视作“本次没有有效数据”。
        unsigned int timestamp = 0;
        if(m_glove == NULL) return 0;
        timestamp = m_glove->GetData(data);
        return timestamp;
    }

    unsigned int wg_getscaleddata(unsigned short* sc_data)
    {
        unsigned int timestamp = 0;
        if(m_glove == NULL) return 0;
        // 一次读取获得 18 路手指传感器原始值。
        timestamp = m_glove->GetData(data_Org);
        if(timestamp>0)
        {
            // 标定期间持续更新每一路传感器的最大值和最小值。
            if(calib_flag)
            {
                update_calibdata(max_data, min_data, data_Org);
            }
            for (int i = 0; i < 18; i++)
            {

                if (data_Org[i] > max_data[i])
                {
                    data_Org[i] = max_data[i];
                }
                if (data_Org[i] < min_data[i])
                {
                    data_Org[i] = min_data[i];
                }

                // 把 [min, max] 线性映射到约 [0, 1000]，便于统一处理。
                sc_data[i] = (data_Org[i] - min_data[i]) * 1000.0 / (max_data[i] - min_data[i]);
            }
        }
        return timestamp;
    }

    unsigned int wg_getangle(float* angle)
    {
        unsigned int timestamp = 0;
        if(m_glove == NULL) return 0;
        timestamp = m_glove->GetAngle(angle);
        return timestamp;
    }

    unsigned int wg_getQuat(float* quat)
    {
        unsigned int timestamp = 0;
        if(m_glove == NULL) return 0;
        timestamp = m_glove->GetQuat(quat);
        return timestamp;
    }

    unsigned int wg_getQuatOrg(float* quat)
    {
        unsigned int timestamp = 0;
        if(m_glove == NULL) return 0;
        timestamp = m_glove->GetQuatOrg(quat);
        return timestamp;
    }
    
    void timer_callback_hand()
    {
        // 本回调是“手指数据支线”，与机械臂 IK 所用的 /quat_data 相互独立。

        // 1) 发布 18 路手指关节角到 /angle_data。
        std_msgs::msg::Float32MultiArray angle_msg;
        angle_msg.data.clear();  // 清空数据
        timestamp = wg_getangle(angle);
        if (timestamp > 0)
        {
            for (int i = 0; i < 18; i++)
            {
                angle_msg.data.push_back(angle[i]);
            }
            angle_publisher_->publish(angle_msg);
            if (timestamp > 0)
            {
                RCLCPP_INFO(this->get_logger(), "Angle Data:");
                for (int i = 0; i < 18; i++)
                {
                    std::cout << std::fixed << std::setprecision(1) <<angle[i]<<" ";
                }
                std::cout<<std::endl;
            }
        }

        // 2) 发布归一化后的 18 路值到 /scaled_data。
        std_msgs::msg::Int32MultiArray scaled_data_msg;
        scaled_data_msg.data.clear();  // 清空数据
        timestamp = wg_getscaleddata(scaled_data);
        if (timestamp > 0)
        {
            for (int i = 0; i < 18; i++)
            {
                scaled_data_msg.data.push_back(scaled_data[i]);
            }
            scaled_data_publisher_->publish(scaled_data_msg);

            RCLCPP_INFO(this->get_logger(), "Scaled Data:");
            for (int i = 0; i < 18; i++)
            {
                std::cout<<scaled_data[i]<<" ";
            }
            std::cout<<std::endl;
        }

        // 3) 发布同一次读取获得的 18 路原始值到 /dataOrg_data。
        std_msgs::msg::Int32MultiArray dataOrg_msg;
        dataOrg_msg.data.clear();  // 清空数据
        if (timestamp > 0)
        {
            for (int i = 0; i < 18; i++)
            {
                dataOrg_msg.data.push_back(data_Org[i]);
            }
            dataOrg_publisher_->publish(dataOrg_msg);

            RCLCPP_INFO(this->get_logger(), "Data:");
            for (int i = 0; i < 18; i++)
            {
                std::cout << std::fixed << std::setprecision(1) << data_Org[i] << " ";
            }
            std::cout<<std::endl;
        }


        // 标定期间在终端显示当前学习到的上下界。
        if (show_calib_data)
        {
            RCLCPP_INFO(this->get_logger(), "--------------------------------");
            RCLCPP_INFO(this->get_logger(), "Max:");
            for (int i = 0; i < 18; i++)
            {
                std::cout<< max_data[i] << " ";
            }
            std::cout<<std::endl;
            RCLCPP_INFO(this->get_logger(), "Min:");
            for (int i = 0; i < 18; i++)
            {
                std::cout <<min_data[i] << " ";
            }
            std::cout<<std::endl;

            RCLCPP_INFO(this->get_logger(), "--------------------------------");
        }
    }

    void timer_callback_arm()
    {
        // sent_quat_flag 初始为 false，所以节点刚启动时 /quat_data 没有消息。
        // 在本节点终端按 'a' 后才会翻转该标志。
        if(!sent_quat_flag) return;

        // Float32MultiArray.data 是动态数组。这里每条消息依次放入 16 个 float，
        // 即 4 组四元数（每组 4 个分量）。后续 arm_tele_real 主要取腕部、
        // 肘部和上臂对应的三组姿态进行逆运动学。
        std_msgs::msg::Float32MultiArray quat_msg;
        quat_msg.data.clear();  // 清空数据
        timestamp = wg_getQuat(quat);
        if (timestamp > 0)
        {
            for (int i = 0; i < 16; i++)
            {
                quat_msg.data.push_back(quat[i]);
            }
            // 本文件最关键的一行：把手套姿态送入机械臂遥操作数据流。
            quat_publisher_->publish(quat_msg);
            RCLCPP_INFO(this->get_logger(), "Quat Data:");
            for (int i = 0; i < 16; i++)
            {
                std::cout << std::fixed << std::setprecision(1) << quat[i] << " ";
            }
            std::cout<<std::endl;
        }

        // /quatOrg_data 是未经 ResetQuat() 参考变换的原始姿态支线，
        // /quat_data 才是教学主链路中 arm_tele_real 使用的数据。
        std_msgs::msg::Float32MultiArray quatOrg_msg;
        quatOrg_msg.data.clear();  // 清空数据
        timestamp = wg_getQuatOrg(quatOrg);
        if (timestamp > 0)
        {
            for (int i = 0; i < 16; i++)
            {
                quatOrg_msg.data.push_back(quatOrg[i]);
            }
            quatOrg_publisher_->publish(quatOrg_msg);

            RCLCPP_INFO(this->get_logger(), "QuatOrg Data:");
            for (int i = 0; i < 16; i++)
            {
                std::cout << std::fixed << std::setprecision(1) << quatOrg[i] << " ";
            }
            std::cout<<std::endl;
        }
    }


    int kbhit(void)
    {
        // 临时把终端设为“非规范、无回显、非阻塞”模式，以检测是否有按键。
        // 读取检查完成后立即恢复原来的终端配置。
        struct termios oldt, newt;
        int ch;
        int oldf;
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);
        oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
        ch = getchar();
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
        fcntl(STDIN_FILENO, F_SETFL, oldf);
        if (ch != EOF)
        {
            ungetc(ch, stdin);
            return 1;
        }
        return 0;
    }

    void glove_calib_callback(const std_msgs::msg::Int8::SharedPtr msg)
    {
        // 非零命令：清空历史边界并开始学习 18 路手指传感器的最大/最小值。
        if(msg->data)
        {
            RCLCPP_INFO(this->get_logger(), "Glove Calib Command Received");
            for(int i = 0; i < 18; i++)
            {
                max_data[i] = 0;
                min_data[i] = USHRT_MAX;
            }
            calib_flag = true;
            show_calib_data = true;
        }
        else
        {
            // 零命令：结束标定，并停止打印标定边界。
            RCLCPP_INFO(this->get_logger(), "Glove Calib Command Stop");
            calib_flag = false;
            show_calib_data = false;
        }
    }

    void keyboard_input_thread()
        {
            // 独立线程每 10 ms 轮询键盘。ROS 2 主线程仍可正常执行定时器和订阅回调。
            while (rclcpp::ok())  
            {
                if (kbhit())
                {
                    char ch = getchar();
                    switch (ch)
                    {
                        case 'q':
                            // q：关闭整个 ROS 2 上下文，随后主线程的 spin() 返回。
                            std::cout << "Exiting..." << std::endl;
                            rclcpp::shutdown();  
                            return;
                        case 'r':
                            // r：开始手指传感器标定，重新学习每一路的最大/最小值。
                            // m_glove->ResetCalib();
                            for(int i = 0; i < 18; i++)
                            {
                                max_data[i] = 0;
                                min_data[i] = USHRT_MAX;
                            }
                            calib_flag = true;
                            show_calib_data = true;
                            break;
                        case 't':
                            // t：结束手指传感器标定。
                            RCLCPP_INFO(this->get_logger(), "Calib Complete");
                            calib_flag = false;
                            show_calib_data = false;
                            break;
                        case 'a':
                            // a：先把当前姿态重置为参考姿态，再切换姿态发布开关。
                            // 第一次按：开始发布 /quat_data；再次按：停止发布。
                            m_glove->ResetQuat();
                            sent_quat_flag = !sent_quat_flag;
                            break;
                        default:
                            break;
                    }
                }

                usleep(10000);  // 10ms
            }
        }

    void update_calibdata(unsigned short* max_data, unsigned short* min_data,unsigned short* dataOrg,size_t size = 50)
    {
        // 用最近 size（默认 50）帧的滑动窗口平滑标定上下界，降低单帧噪声影响。
        if (max_vec.size() < size)
        {
            std::array<unsigned short, 18> max_data_;
            for (int i = 0; i < 18; i++)
            {
                max_data_[i] = dataOrg[i];
            }
            max_vec.push_back(max_data_);
        }
        if (min_vec.size() < size)
        {
            std::array<unsigned short, 18> min_data_;
            for (int i = 0; i < 18; i++)
            {
                if (data_Org[i] < 200.0){continue;}
                min_data_[i] = dataOrg[i];
            }
            min_vec.push_back(min_data_);
        }
        std::array<unsigned short, 18> max_data_avg;
        std::array<unsigned short, 18> min_data_avg;
        for(size_t i = 0; i < 18 ; i++)
        {
            int max_data_sum = 0;
            int min_data_sum = 0;
            for (size_t j = 0; j < max_vec.size(); j++)
            {
                max_data_sum += max_vec[j][i];
            }
            max_data_avg[i] = max_data_sum / max_vec.size();
            if (max_data_avg[i] > max_data[i])
            {
                max_data[i] = max_data_avg[i];
            }
            for(size_t j = 0; j < min_vec.size(); j++)
            {
                min_data_sum += min_vec[j][i];
            }
            min_data_avg[i] = min_data_sum / min_vec.size();
            if (min_data_avg[i] < min_data[i])
            {
                min_data[i] = min_data_avg[i];
            }
        }
        if(max_vec.size() >= size)
        {
            max_vec.erase(max_vec.begin());
        }
        if(min_vec.size() >= size)
        {
            min_vec.erase(min_vec.begin());
        }

        for(int i = 0; i < 10; i++)
        {
            for (int j = 0; j < 18; j++)
            {
                std::cout <<std::setw(3)<< max_vec[i][j] << " ";
            }
            std::cout<<std::endl;
        }
        for(int i = 0; i < 10; i++)
        {
            for (int j = 0; j < 18; j++)
            {
                std::cout <<std::setw(3)<< min_vec[i][j] << " ";
            }
            std::cout<<std::endl;
        }
    }


    // ------------------------- 硬件数据与状态 -------------------------
    Wiseglove* m_glove = NULL;               // 手套硬件接口
    unsigned short scaled_data[18] = {0};
    unsigned short data_Org[18] = {0};       // 18 路手指传感器原始值

    unsigned short max_data[18] = {0};
    unsigned short min_data[18] = {USHRT_MAX,USHRT_MAX,USHRT_MAX,USHRT_MAX,
                                USHRT_MAX,USHRT_MAX,USHRT_MAX,USHRT_MAX,
                                USHRT_MAX,USHRT_MAX,USHRT_MAX,USHRT_MAX,
                                USHRT_MAX,USHRT_MAX,USHRT_MAX,USHRT_MAX,
                                USHRT_MAX,USHRT_MAX};

    std::vector<std::array<unsigned short, 18>> max_vec;
    std::vector<std::array<unsigned short, 18>> min_vec;


    // 这些标志会被键盘线程和 ROS 2 回调共同访问，因此使用 atomic 避免数据竞争。
    std::atomic<bool> calib_flag = false;       // 是否正在标定手指传感器
    std::atomic<bool> show_calib_data = false;  // 是否打印标定上下界
    std::atomic<bool> sent_quat_flag = false;   // 是否允许发布姿态（初始禁止）

    float angle[18] = {0};     // 18 路手指角度
    float quat[16] = {0};      // 4 组经过参考姿态处理的四元数
    float quatOrg[16] = {0};   // 4 组原始四元数
    char model[20];
    char sn[20];
    unsigned int timestamp = 0;

    // ROS 2 通信对象必须保存为成员，否则构造函数结束后会被销毁。
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr angle_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr scaled_data_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr dataOrg_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr quat_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr quatOrg_publisher_;

    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr glove_calib_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr glove_imu_calib_subscriber_;

    rclcpp::TimerBase::SharedPtr timer_arm_hz;   // 四元数支线，20 ms
    rclcpp::TimerBase::SharedPtr timer_hand_hz;
    std::thread keyboard_thread_;                // 非阻塞键盘监听

};





int main(int argc, char **argv)
{
    // 初始化 ROS 2，解析 ROS 参数和 remap 等命令行参数。
    rclcpp::init(argc, argv);

    // 创建节点并进入事件循环；定时器回调和订阅回调都由 spin() 调度。
    auto node = std::make_shared<GloveDataPub>();
    rclcpp::spin(node);
    // spin() 通常在按 q 或收到系统退出信号后返回。
    rclcpp::shutdown();
    return 0;
}
