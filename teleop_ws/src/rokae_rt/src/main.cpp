#include <iostream>
#include <cmath>
#include <thread>
#include <mutex> // 用于线程同步
#include <array>
#include <termios.h>
#include <fcntl.h>
#include <deque>
#include <chrono>

#include "rokae_rt/robot.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"


using namespace std::chrono_literals;
using namespace rokae;

/*
 * Rokae 真机实时控制节点
 * =====================
 *
 * 教学主链路的最后一段：
 *   /sent_joints（MPC 输出的 7 个高频关节命令）
 *       -> jointPositionCallback()：校验并放入有限队列
 *       -> rokae_callback()：实时循环逐条取出
 *       -> Rokae 实时控制 SDK
 *       -> xMateEr Pro 物理机械臂
 *
 * 安全边界：
 *   本文件会连接、切换模式、上电并实际移动机械臂。启动节点本身就会在
 *   构造函数中执行一次 MoveJ 到 zero_pos；按下 'c' 只是之后再启动实时
 *   关节位置跟随，不能把“尚未按 c”理解为“机器人绝对不会运动”。
 */

class rt_RobotCtrlNode :  public rclcpp::Node
{
public:
    rt_RobotCtrlNode() : Node("rt_robot_control_node")
    {
        // 连接真机：robot_ip 是机械臂控制器地址，local_ip 是本机控制网卡地址。
        std::error_code ec;
        try
        {
            std::string robot_ip = "192.168.0.160";
            std::string local_ip = "192.168.0.100";
            robot_.connectToRobot(robot_ip, local_ip);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect to robot: %s", e.what());
            rclcpp::shutdown();
            return;
        }
        // 进入自动/实时命令模式并上电。这些调用已经改变物理设备状态。
        robot_.setOperateMode(OperateMode::automatic, ec);
        robot_.setRtNetworkTolerance(20, ec);
        // robot_.setMotionControlMode(MotionControlMode::NrtCommand, ec);
        robot_.setMotionControlMode(MotionControlMode::RtCommand, ec);
        robot_.setPowerState(true,ec);

        // 订阅 MPC 的 /sent_joints。只保留最新一条，并使用可靠、易失 QoS。
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1))
        .reliability(rclcpp::ReliabilityPolicy::Reliable)
        .durability(rclcpp::DurabilityPolicy::Volatile)
        .deadline(rclcpp::Duration(1ms));

        joint_positions_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "sent_joints", qos, 
            std::bind(&rt_RobotCtrlNode::jointPositionCallback, this, std::placeholders::_1)
        );

        /*
         * 启动阶段的物理动作：
         * 获取实时控制器、以 1 ms 周期接收机器人状态，然后调用 MoveJ 回零。
         * 这段代码发生在构造函数里，早于键盘线程，也不需要先按 'c'。
         */
        try
        {
            RCLCPP_INFO(this->get_logger(), "Initializing robot joint positions to zero.");
            motion_controller_ = robot_.getRtMotionController().lock();

            // motion_controller_->setCollisionBehaviour(torqueThresholds,ec);
            robot_.startReceiveRobotState(std::chrono::milliseconds(1),{RtSupportedFields::jointPos_m});

            std::array<double, 7> cur_pos {};
            robot_.getStateData(RtSupportedFields::jointPos_m, cur_pos);
            
            std::cout << "Current joint positions: ";
            for (double value : cur_pos) {
                std::cout << value << " ";
            }
            std::cout << std::endl;

            // PowerState state = robot_.powerState(ec);
            // std::string state_str;
            // switch (state)
            // {
            //     case PowerState::on:
            //         state_str = "上电";
            //         break;
            //     case PowerState::off:
            //         state_str = "下电";
            //         break;
            //     case PowerState::unknown:
            //         state_str = "未知";
            //         break;
            //     case PowerState::estop:
            //         state_str = "急停";
            //         break;
            //     case PowerState::gstop:
            //         state_str = "安全门打开";
            //         break;
            //     default:
            //         state_str = "无效状态";
            //         break;
            // }
            // std::cout << "Robot power state: " << state_str << std::endl;
            
            // 从当前关节位置运动到 7 轴全零位；这是实际真机运动指令。
            motion_controller_->MoveJ(0.5,robot_.jointPos(ec),zero_pos);
            RCLCPP_INFO(this->get_logger(), "Robot joint positions initialized to zero.");

            // std::error_code ec;

            // std::array<double,7> joint_torque_measured,external_torque_measured;
            // std::array<double, 3> cart_torque,cart_force;
            // robot_.getEndTorque(FrameType::world,joint_torque_measured,external_torque_measured,cart_torque,cart_force,ec);
            
            // std::cout << "Current ex_tq : ";
            // for (double value : external_torque_measured) {
            //     std::cout << value << " ";
            // }
        }
        catch(std::exception &e)
        {
            std::cerr << e.what();
        }



        // 键盘线程用于启动控制模式或退出，避免阻塞 ROS 2 订阅回调。
        keyboard_thread_ = std::thread([this]()
        {
            this->keyboard_input_thread();
        });
    };

    ~rt_RobotCtrlNode()
    {
        // 节点退出时停止实时循环，随后下电并退回 Idle 模式。
        if(control_thread_.joinable())
        {
            motion_controller_->stopLoop();
            control_thread_.join();
            if (keyboard_thread_.joinable())
                keyboard_thread_.join();
        }
        std::error_code ec;
        robot_.setPowerState(false, ec);
        robot_.setMotionControlMode(MotionControlMode::Idle,ec);
        std::cout<< "Robot power off." << std::endl;
    }

private:

    void jointPositionCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        // 驱动只接受恰好 7 个元素的消息；其他长度不会进入控制队列。
        if(msg->data.size() == 7)
        {
            std::array<double, 7> new_positions;
            for (size_t i = 0; i < 7; i++)
            {
                new_positions[i] = static_cast<double>(msg->data[i]);
            }
            {
                std::lock_guard<std::mutex> lock(joint_positions_mutex_);
                // 队列达到上限时丢弃最旧命令，避免延迟无限累积。
                if(joint_queue_.size() >= max_queue_size_) {
                    joint_queue_.pop_front();
                }
                joint_queue_.push_back(new_positions);
            }
        }

        // 调试：/sent_joints 设计周期约 1 ms，间隔超过 1.5 ms 时报警。
        static rclcpp::Time last_msg_time = this->get_clock()->now();
        rclcpp::Time current_msg_time = this->get_clock()->now();
        double interval_ms = (current_msg_time - last_msg_time).seconds() * 1000.0;
        last_msg_time = current_msg_time;
        if (interval_ms > 1.5) 
        {
            RCLCPP_WARN(this->get_logger(), "Large message interval detected: %.4f ms", interval_ms);
        }
    }

    

    JointPosition rokae_callback()
    {
        // 此函数由 Rokae SDK 的实时控制循环反复调用，每次必须返回一个目标。
        std::error_code ec;
        bool is_ready_to_move = false;
        bool has_new_command = false;
        std::array<double,7> current_target_joint_pos_ {};
        {
            std::lock_guard<std::mutex> lock(joint_positions_mutex_);
            if(!joint_queue_.empty()) 
            {
                // 有新命令：按先进先出顺序取出，并记为最近一次有效命令。
                current_target_joint_pos_ = joint_queue_.front();
                joint_queue_.pop_front();
                last_valid_command_ = current_target_joint_pos_;
                has_new_command = true;
            }
            if(!init_joint_pos_set_ && init_move_completed)
            {
                init_joint_pos_set_ = true;
            }
            is_ready_to_move = init_joint_pos_set_ ;
        }

        JointPosition cmd;
        // std::cout << init_joint_pos_set_<< std::endl;
        if(is_ready_to_move && has_new_command)
        {
            // 初始化动作已完成且队列有数据：执行新目标。
            cmd.joints = std::vector<double>(current_target_joint_pos_.begin(), current_target_joint_pos_.end());
        }
        else if(is_ready_to_move &&  !has_new_command)
        {
            // 暂时断流：保持最近一次有效目标，而不是生成随机或空命令。
            cmd.joints = std::vector<double>(last_valid_command_.begin(), last_valid_command_.end());
        }
        else
        {
            // 实时跟随尚未就绪：命令当前位置，使机械臂保持不动。
            std::array<double,7> cur_jntPos = robot_.jointPos(ec);
            cmd.joints = std::vector<double>(cur_jntPos.begin(),cur_jntPos.end());
        }
        // for (double value : cmd.joints) {
        //         std::cout << value << " ";
        //     }
        //     std::cout << std::endl;
        return cmd;
    }



    int kbhit(void)
    {
        // 非阻塞检测终端按键，并在检查后恢复终端属性。
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

    void keyboard_input_thread()
    {
        while (rclcpp::ok())  
        {
            if (kbhit())
            {
                char ch = getchar();
                switch (ch)
                {
                    case 'c':
                        // c：启动关节位置实时跟随。
                        followed_loop(RtControllerMode::jointPosition);
                        break;
                    case 'v':
                        // v：启动关节阻抗实时跟随。
                        followed_loop(RtControllerMode::jointImpedance);
                        break;
                    case 'q':
                        // q：请求 ROS 2 关闭；析构函数随后停止循环并下电。
                        std::cout << "Exiting..." << std::endl;
                        rclcpp::shutdown();  
                        return;
                    default:
                        break;
                }
            }
            std::this_thread::sleep_for(10ms);
        }
    }

    void followed_loop(RtControllerMode followed_mode)
    {
        // 只允许启动一次，避免重复创建 SDK 控制循环和控制线程。
        if (!control_loop_started_) 
        {
            std::error_code ec;
            std::array<double, 7> target_pos {};
            {
                std::lock_guard<std::mutex> lock(joint_positions_mutex_);
                target_pos = joint_queue_.empty() ? zero_pos : joint_queue_.back();
                joint_queue_.clear();
            }

            std::cout << "Current target positions: ";
            for (double value : target_pos) {
                std::cout << value << " ";
            }
            std::cout << std::endl;

            if (!motion_controller_)
            {
                RCLCPP_FATAL(this->get_logger(),"Motion Controller error!");
                rclcpp::shutdown();
                return;
            }

            // 启动实时循环前，先 MoveJ 到队列最新目标（无消息时为零位），
            // 让实际起点与即将执行的命令尽量衔接；这也是一次物理运动。
            motion_controller_->MoveJ(0.5,robot_.jointPos(ec),target_pos);
            // size_t size_q = 0;
            // {
            //     std::lock_guard<std::mutex> lock(joint_positions_mutex_);
            //     size_q = joint_queue_.size();
            // }
            // std::cout << size_q << std::endl;

            if(followed_mode == RtControllerMode::jointImpedance)
            {
                RCLCPP_INFO(this->get_logger(),"Set joint impedance control mode");
                motion_controller_->setJointImpedance({500, 500, 500, 500, 50, 50, 50}, ec);
                
            }
            else if(followed_mode == RtControllerMode::jointPosition)
            {
                RCLCPP_INFO(this->get_logger(),"Set joint position control mode");
            }
            else
            {
                RCLCPP_INFO(this->get_logger(),"This mode is unsupported now,exiting...");
                rclcpp::shutdown();
                return;
            }

            motion_controller_->startMove(followed_mode);
            {
                std::lock_guard<std::mutex> lock(joint_positions_mutex_);
                init_move_completed = true;
            }

            // 注册实时回调：SDK 每个控制周期通过 rokae_callback() 取得目标。
            motion_controller_->setControlLoop(
                std::function<JointPosition()>(std::bind(&rt_RobotCtrlNode::rokae_callback, this)),
                0,
                true
            );
            RCLCPP_INFO(this->get_logger(), "Control loop started.");
            // startLoop() 是阻塞式实时循环，因此在独立线程中运行。
            control_thread_ = std::thread([this]() 
            {
                try 
                {
                    // t0 = std::chrono::high_resolution_clock::now();
                    this->motion_controller_->startLoop(true);
                } catch (const std::exception& e) 
                {
                    RCLCPP_ERROR(this->get_logger(), "startLoop exception: %s", e.what());
                }
            });
            control_loop_started_ = true;
        }
    }


    // -------------------- 硬件接口与 ROS 通信 --------------------
    rokae::xMateErProRobot robot_;

    std::shared_ptr<RtMotionControlCobot<7>> motion_controller_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr joint_positions_sub_;

    // rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr filted_joints;
    
    const std::array<double, 7> zero_pos = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};


    const std::array<double, 7> torqueThresholds = { 75, 75, 60, 45, 30, 30, 20 };
    std::array<double, 7> last_valid_command_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    // ROS 订阅线程与 SDK 实时线程共享命令队列，访问时必须加锁。
    std::mutex joint_positions_mutex_;
    std::deque<std::array<double,7>> joint_queue_;
    const size_t max_queue_size_ = 100; // 定长队列大小


    // 三个状态分别表示：可从队列取目标、启动前 MoveJ 已完成、实时循环已启动。
    bool init_joint_pos_set_ = false;
    bool init_move_completed = false;
    bool control_loop_started_ = false;

    std::thread control_thread_;
    std::thread keyboard_thread_; 
    std::chrono::time_point<std::chrono::high_resolution_clock> t0;
    int count = 0;

};


int main(int argc, char **argv)
{
    // 创建节点就会连接、上电并回零；spin() 负责接收 /sent_joints。
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rt_RobotCtrlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
