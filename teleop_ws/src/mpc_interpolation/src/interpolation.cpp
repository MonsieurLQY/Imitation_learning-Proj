#include <iostream>
#include <chrono>
#include <array>
#include <sstream> 
#include <mutex>
#include <pthread.h>
#include <sched.h>
#include <unistd.h>
#include <queue>
#include <fstream>

#include <boost/lockfree/queue.hpp>
#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "mpc_interpolation/mpc_spline.hpp"
#include "mpc_interpolation/cubic_spline.hpp"

/*
 * MPC 关节轨迹插值节点
 * ===================
 *
 * 教学主链路：
 *   /rokae_control_joints（IK 给出的 7 个低频目标关节角）
 *       -> update_ref_points()：接收并低通滤波
 *       -> exec_planning()：在相邻目标间生成参考点
 *       -> mpcTrajCompute()：考虑位置、速度、加速度等约束进行平滑
 *       -> mimicSendGoal()：以 1000 Hz 发布 7 个关节命令
 *       -> /sent_joints
 *
 * 这里的三个工作线程通过无锁队列串联。理解数据方向和各线程频率比一开始
 * 深究 MPC 数学推导更重要；权重来自 config/mpc_weight.yaml。
 */

constexpr int jointSize = 7;
constexpr double mpc_dt = 0.001; // MPC 离散时间 1 ms，对应 1000 Hz
// 启动闸门：先积累参考点/MPC 输出，避免下游线程一启动就读到空队列。
constexpr uint32_t MPC_START_REF = 40;
constexpr uint32_t MIMIC_START_MPC_NUM = 10;
// constexpr size_t MPC_BUFFER_CAP = 16;
using Vector7d = Eigen::Vector<real_t, jointSize>;
using Matrix7H = Eigen::Matrix<real_t, jointSize, Horizon>;
using JointArray = std::array<double, jointSize>;

using namespace std::chrono_literals;

bool setRealtimePriority(pthread_t thread, int priority)
{
    // 尝试使用 Linux SCHED_FIFO 实时调度；权限不足时只报错，不终止节点。
    sched_param sch_params;
    sch_params.sched_priority = priority;

    if (pthread_setschedparam(thread, SCHED_FIFO, &sch_params)) 
    {
        std::cerr << "Failed to set thread priority (need CAP_SYS_NICE)." << std::endl;
        return false;
    }
    return true;
}

bool setThreadAffinity(pthread_t thread, int cpu_core_id)
{
    // 把高频线程固定到指定 CPU 核，减少调度迁移带来的时间抖动。
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(cpu_core_id, &cpuset);

    if (pthread_setaffinity_np(thread, sizeof(cpu_set_t), &cpuset)) 
    {
        std::cerr << "Failed to set thread affinity to CPU " << cpu_core_id << std::endl;
        return false;
    }
    return true;
}

class interpolation : public rclcpp::Node
{
public:

    explicit interpolation() : Node("interpolation_node"),running_(true)
    {
        mpc_exec_times_.reserve(MAX_LOG_SAMPLES);
        // Initialize the node
        RCLCPP_INFO(this->get_logger(), "Interpolation node started.");

        // 主链路只关心最新的实时目标，因此历史深度为 1。
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1))
            .reliability(rclcpp::ReliabilityPolicy::Reliable)
            .durability(rclcpp::DurabilityPolicy::Volatile)
            .deadline(rclcpp::Duration(1ms));

        // 上游 arm_tele_real 的输出；每条消息预期恰好包含 7 个关节角。
        roake_control_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/rokae_control_joints", qos, std::bind(&interpolation::update_ref_points, this,std::placeholders::_1));
        

        // 四类代价权重各有 7 个值，对应机器人 7 个关节。
        // launch 加载 YAML 时会覆盖这些默认值。
        std::vector<double> default_pos(7, 4.05);
        std::vector<double> default_vel(7, 0.03);
        std::vector<double> default_acc(7, 0.0000125);
        std::vector<double> default_ctrl(7, 8e-9);

        this->declare_parameter<std::vector<double>>("mpc_weights.position", default_pos);
        this->declare_parameter<std::vector<double>>("mpc_weights.velocity", default_vel);
        this->declare_parameter<std::vector<double>>("mpc_weights.acceleration", default_acc);
        this->declare_parameter<std::vector<double>>("mpc_weights.control", default_ctrl);

        // 从 ROS 参数服务器读取 config/mpc_weight.yaml 中的权重。
        pos_weights_ = this->get_parameter("mpc_weights.position").as_double_array();
        vel_weights_ = this->get_parameter("mpc_weights.velocity").as_double_array();
        acc_weights_ = this->get_parameter("mpc_weights.acceleration").as_double_array();
        ctrl_weights_ = this->get_parameter("mpc_weights.control").as_double_array();

        MPCsplines_.reserve(jointSize);
        MPCsplines_.clear();

        // 每个关节各创建一个一维 MPC 求解器，七个求解结果再组合成 7 维命令。
        for (size_t i = 0; i < jointSize; ++i) 
        {
            MPCsplines_.emplace_back(
                mpc_dt, 
                i, 
                pos_weights_[i], 
                vel_weights_[i], 
                acc_weights_[i], 
                ctrl_weights_[i]
            );
            std::cout << "Joint_" << i << ": " << "pos_w:" << pos_weights_[i]
            << "; vel_w: " << vel_weights_[i] << "; acc_w: " << acc_weights_[i] <<"; ctrl_w: " << ctrl_weights_[i]<<std::endl;
        }

        // 线程 1：50 Hz 生成线性插值参考轨迹。
        exec_plan_thread_ = std::thread(&interpolation::exec_planning,this);
        pthread_t handle_exec = exec_plan_thread_.native_handle();
        setRealtimePriority(handle_exec,85);
        setThreadAffinity(handle_exec,1);

        // 线程 2：1000 Hz 滚动求解 MPC。
        mpc_thread_ = std::thread(&interpolation::mpcTrajCompute, this);
        //  设置 mpc_thread_ 为实时线程 + 绑定 CPU core1
        pthread_t handle_mpc = mpc_thread_.native_handle();
        setRealtimePriority(handle_mpc,85);
        setThreadAffinity(handle_mpc,2);

        // 教学主链路的第三个关键 Topic：rokae_rt_node 订阅 /sent_joints。
        mimic_send_goal_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "sent_joints", qos);
        // /sent_cube_ref 仅用于观察未经 MPC 平滑的参考轨迹。
        mimic_send_cuberef_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "sent_cube_ref", qos);

        // 线程 3：1000 Hz 从结果队列取点并发布。
        mimic_thread_ = std::thread(&interpolation::mimicSendGoal, this);

        //  设置 mimic_thread_ 为实时线程 + 绑定 CPU core2
        pthread_t handle = mimic_thread_.native_handle();
        setRealtimePriority(handle, 75);  // 建议 40~70 之间，不要太高
        setThreadAffinity(handle, 3);     // 绑到 CPU core2
    }

    ~interpolation()
    {
        running_ = false;
        if (mpc_exec_times_.size() > 0) {
            // saveExecTimesToFile();
        }
        if (mpc_thread_.joinable()) mpc_thread_.join();
        if (mimic_thread_.joinable())mimic_thread_.join();
        if (exec_plan_thread_.joinable())mimic_thread_.join();
    }

private:


    void update_ref_points(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        // 把上游消息的 7 个 float 转成 Eigen 7 维向量。
        Vector7d new_target = conv2EigenVec(msg->data);
        {
            std::lock_guard<std::mutex> lk(target_mtx_);
            // 一阶低通：新目标占 alpha=0.8，旧滤波值占 0.2，
            // 用于减小 IK 目标的突变。互斥锁保护多个线程共享的目标。
            if (!filter_initialized_) {
                filtered_target_ = new_target;
                filter_initialized_ = true;
            } else {
                filtered_target_ =
                    target_alpha_ * new_target +
                    (1.0 - target_alpha_) * filtered_target_;
            }
            end_position_ = filtered_target_;
            target_updated_ = true;
        }
    }

    void exec_planning()
    {
        // 与上游手套/IK 的约 50 Hz 更新节奏一致。
        rclcpp::Rate rate(50);

        Vector7d local_end_position;
        while(rclcpp::ok() && running_)
        {
            {
                std::lock_guard<std::mutex> lk(target_mtx_);
                if (target_updated_)
                {
                    local_end_position = end_position_;
                    target_updated_ = false;
                }
            }
            /* ------------------三次曲线插值----------------- */
            // CubicSplineTrajectoryPlanner planner(
            //     start_position_, start_velocity_, local_end_position, IPT_NUM);
            // planner.coeffs = planner.computeCubicCoefficients();
            // for (int i = 0; i < IPT_NUM; ++i)
            // {
            //     double t = static_cast<double>(i) / (IPT_NUM - 1);
            //     JointArray pt{};
            //     Vector7d eigen_pt = planner.evaluatePolynomial(t, planner.coeffs);
            //     for (int j = 0; j < jointSize; ++j)
            //     {
            //         pt[j] = eigen_pt(j);
            //     }
            //     if (ref_traj_queue_.push(pt)) 
            //     {
            //         ref_produced_.fetch_add(1, std::memory_order_relaxed);
            //     }
            // }

            /* ---------------线性插值----------------------
             * 在上一个目标 start_position_ 和最新目标 local_end_position 之间，
             * 均匀生成 IPT_NUM=20 个点并放入 ref_traj_queue_，供 MPC 消费。
             */
            for (int i = 0; i < IPT_NUM; ++i)
            {
                double alpha = static_cast<double>(i) / (IPT_NUM - 1);

                Vector7d eigen_pt =
                    (1.0 - alpha) * start_position_ +
                    alpha * local_end_position;

                JointArray pt{};
                for (int j = 0; j < jointSize; ++j)
                {
                    pt[j] = eigen_pt(j);
                }

                if (ref_traj_queue_.push(pt))
                {
                    ref_produced_.fetch_add(1, std::memory_order_relaxed);
                }
            }



            if (!mpc_enabled_.load(std::memory_order_acquire) &&
                ref_produced_.load(std::memory_order_relaxed) >= MPC_START_REF)
            {
                // 至少积累 40 个参考点后才允许 MPC 开始。
                mpc_enabled_.store(true, std::memory_order_release);
            }

            start_position_ = local_end_position;
            // start_velocity_ = planner.evaluatePolynomial(1.0, planner.coeffs, 1); // 三次曲线特有

            rate.sleep();
        }
    }

    void mimicSendGoal()
    {
        // 高频发布线程：目标周期 1 ms。实际频率仍受系统调度和计算耗时影响。
        rclcpp::Rate rate(1000);

        while (rclcpp::ok() && running_)
        {
            if (!mimic_enabled_.load(std::memory_order_acquire)) 
            {
                rate.sleep();
                continue;
            }

            JointArray cmd;
            if (!mpc_traj_queue_.pop(cmd)) 
            {
                // 这里理论上不应该发生
                RCLCPP_ERROR_THROTTLE(
                    this->get_logger(), *this->get_clock(), 1000,
                    "mimic enabled but mpc_traj_queue empty!"
                );
                rate.sleep();
                continue;
            }

            // MPC 结果固定打包为 7 个 float 并发布到 /sent_joints。
            std_msgs::msg::Float32MultiArray msg;
            msg.data.resize(jointSize);
            for (int i = 0; i < jointSize; ++i)
                msg.data[i] = static_cast<float>(cmd[i]);
            mimic_send_goal_->publish(msg);

            JointArray cmd_ref;

            if (!cube_traj_.pop(cmd_ref)) 
            {
                // 这里理论上不应该发生
                RCLCPP_ERROR_THROTTLE(
                    this->get_logger(), *this->get_clock(), 1000,
                    "mimic enabled but cube_traj_ empty!"
                );
                rate.sleep();
                continue;
            }
            std_msgs::msg::Float32MultiArray cube_msg;
            cube_msg.data.resize(jointSize);
            for (int i = 0; i < jointSize; ++i)
                cube_msg.data[i] = static_cast<float>(cmd_ref[i]);
            mimic_send_cuberef_->publish(cube_msg);


            // static uint64_t seq = 0;
            // auto now = std::chrono::high_resolution_clock::now();
            // auto us = std::chrono::duration_cast<std::chrono::microseconds>(
            //             now.time_since_epoch()).count();

            // RCLCPP_INFO(this->get_logger(),
            //     "PUB seq=%lu time=%ld us q0=%.6f",
            //     seq++, us, cmd_ref[0]);

            rate.sleep();
        }
    }

    /**
     * @brief 使用滚动时域方式计算 MPC 轨迹。
     * 等待足够的参考点，以当前位置/速度/加速度为状态，为 7 个关节分别求解，
     * 再把下一步 7 维位置送入 mpc_traj_queue_。
     */
    void mpcTrajCompute()
    {
        rclcpp::Rate rate(1000);
        std::deque<JointArray> ref_window;


        while(rclcpp::ok() && running_)
        {

            // uint64_t prod = ref_produced_.load();
            // uint64_t cons = ref_consumed_.load();

            if (!mpc_enabled_.load(std::memory_order_acquire)) 
            {
                // printf("[REF STAT] produced=%lu consumed=%lu\n",
                // prod, cons);
                rate.sleep();
                continue;
            }
            // printf("[REF STAT] produced=%lu consumed=%lu in_queue≈%ld\n",
            //     prod, cons, (long)(prod - cons));
            auto t0 = std::chrono::high_resolution_clock::now();            
            JointArray raw{};
            Vector7d pt;
            while (ref_window.size() < Horizon && ref_traj_queue_.pop(raw))
            {
                // 凑齐长度为 Horizon 的预测窗口。
                ref_window.push_back(raw);
                cube_traj_.push(raw);
                ref_consumed_.fetch_add(1, std::memory_order_relaxed);
            }

            if (ref_window.size() < Horizon) 
            {
                printf("ref windows less than 5\n");
                rate.sleep();
                continue;
            }

            // 把参考窗口整理为“7 个关节 × Horizon 个时刻”的矩阵。
            Matrix7H ref_horizon;
            for (int i = 0; i < Horizon; ++i)
            {
                for (int j = 0; j < jointSize; ++j)
                {
                    ref_horizon(j, i) = ref_window[i][j];
                }
            }

            Vector7d x0 = first_mpc_run_ ? ref_horizon.col(0) : last_pos_;
            if (first_mpc_run_) {
                last_vel_.setZero();
                last_acc_.setZero();
                first_mpc_run_ = false;
            }
            
            Matrix7H mpc_out;
            bool mpc_compute_ok = true;
            // 七个关节逐个求解；代价中同时考虑位置、速度、加速度和控制量。
            for (int j = 0; j < jointSize; ++j)
            {
                Eigen::Vector3d state0;
                state0 << x0(j), last_vel_(j), last_acc_(j);

                MPCsplines_[j].setCurrentState(state0);
                MPCsplines_[j].setReferenceTrajectory(ref_horizon.row(j));

                if (!MPCsplines_[j].computeMPC()) {
                    mpc_compute_ok = false;
                    RCLCPP_ERROR(this->get_logger(),"MPC_%d Compute Failed!",j);
                    break;
                }
                mpc_out.row(j) = MPCsplines_[j].getPrediction().transpose();
                auto full = MPCsplines_[j].getFullStatePrediction();
                
                // Eigen::IOFormat fmt(
                //     Eigen::StreamPrecision,
                //     Eigen::DontAlignCols,
                //     ", ",
                //     ", ",
                //     "[",
                //     "]"
                // );

                // std::cout << "ref traj:" << "(joint"<< j  << ") "<< ref_horizon.row(j) << std::endl;
                // std::cout << "real traj:(joint" << j << ") "
                //         << full.transpose().format(fmt)
                //         << std::endl;

                last_vel_(j) = full(1);
                last_acc_(j) = full(2);
            }
            if (mpc_compute_ok)
            {
                // N_apply=1：每次只采用本轮预测轨迹的第一个新点，
                // 下一轮再根据更新后的状态重新优化，这就是“滚动时域”。
                JointArray mpc_out_arr{};
                for(int j = 0;j<jointSize;j++)
                {
                    mpc_out_arr[j] = mpc_out(j,N_apply - 1);
                }

                if(mpc_traj_queue_.push(mpc_out_arr))
                {
                    mpc_produced_.fetch_add(1, std::memory_order_relaxed);
                }
                if (!mimic_enabled_.load(std::memory_order_acquire) &&
                    mpc_produced_.load(std::memory_order_relaxed) >= MIMIC_START_MPC_NUM)
                {
                    // 先缓存 10 个 MPC 结果，再放行 1000 Hz 发布线程。
                    mimic_enabled_.store(true, std::memory_order_release);
                }

                last_pos_ = mpc_out.col(N_apply - 1);
                for (unsigned i = 0; i < N_apply; ++i)
                {    
                    ref_window.pop_front();
                }
            }
            else
            {
                // RCLCPP_ERROR(this->get_logger(),"MPC Compute Failed!");
            }

            auto dt = std::chrono::duration_cast<std::chrono::microseconds>(
                          std::chrono::high_resolution_clock::now() - t0)
                          .count();
            if (mpc_exec_times_.size() < MAX_LOG_SAMPLES)
                        {
                            mpc_exec_times_.push_back(dt);
                        }
            else if (!dt_log_saved_.load())
            {
                // 凑满 10万次了，直接写文件
                // saveExecTimesToFile(); 
            }
            
            if (dt > 1000)
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                     "MPC loop took %ld us", dt);

            rate.sleep();
        }
    }

    std::stringstream lst2stream(const std::vector<float>& lst)
    {
        std::stringstream ss;
        ss << "[";
        for (size_t i = 0; i < lst.size(); ++i) 
        {
            ss << lst[i];
            if (i != lst.size() - 1) {
                ss << ", ";
            }
        }
        ss << "]";
        return ss;
    }

    Vector7d conv2EigenVec(const std::vector<float>& lst)
    {
        // 主链路约定输入长度为 7；上游 arm_tele_real 正常发布 7 个关节角。
        Vector7d out;
        for (size_t i = 0; i < lst.size(); ++i) out(i)= lst[i];
        return out;
    }

    const int IPT_NUM = 20; // 每组相邻低频目标之间生成的参考点数
    // ===================== 无锁队列 =====================
    // 数据方向：50 Hz 插值线程 → MPC 线程
    boost::lockfree::queue<JointArray, boost::lockfree::capacity<8192>> ref_traj_queue_;
    // 数据方向：MPC 线程 → 1000 Hz 发布线程
    boost::lockfree::queue<JointArray, boost::lockfree::capacity<4096>> mpc_traj_queue_;
    // boost::lockfree::queue<JointArray,boost::lockfree::capacity<MPC_BUFFER_CAP>> mpc_buffer_queue_;

    boost::lockfree::queue<JointArray, boost::lockfree::capacity<4096>> cube_traj_;

    // ===================== 状态 =====================
    Vector7d start_position_ = Vector7d::Zero();
    Vector7d start_velocity_ = Vector7d::Zero();
    Vector7d end_position_ = Vector7d::Zero();
    std::mutex target_mtx_;
    bool target_updated_{false};

    Vector7d last_pos_ = Vector7d::Zero();
    Vector7d last_vel_ = Vector7d::Zero();
    Vector7d last_acc_ = Vector7d::Zero();
    bool first_mpc_run_ = true;

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr roake_control_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr mimic_send_goal_;

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr mimic_send_cuberef_;

    std::vector<double> pos_weights_;
    std::vector<double> vel_weights_;
    std::vector<double> acc_weights_;
    std::vector<double> ctrl_weights_;
    std::vector<MpcSpline<Horizon, InputNum>> MPCsplines_;

    std::atomic<bool> running_;
    std::thread mpc_thread_, mimic_thread_,exec_plan_thread_;

    std::atomic<bool> mpc_enabled_{false};
    std::atomic<uint32_t> ref_produced_{0};
    std::atomic<uint64_t> ref_consumed_{0};

    std::atomic<uint32_t> mpc_produced_{0};    // MPC 已产生的点数
    std::atomic<bool> mimic_enabled_{false};  // mimic 启动闸门

    const unsigned int N_apply = 1; // 每轮 MPC 只执行预测序列中的 1 步

    // /rokae_control_joints 目标的一阶低通滤波状态
    Vector7d filtered_target_ = Vector7d::Zero();
    bool filter_initialized_ = false;
    const double target_alpha_ = 0.8;

// ===================== 压测与耗时记录 =====================
    std::vector<int64_t> mpc_exec_times_;
    const size_t MAX_LOG_SAMPLES = 100000; // 记录 10万条数据 (1000Hz 下等于 100 秒)
    std::atomic<bool> dt_log_saved_{false};

    void saveExecTimesToFile()
    {
        if (dt_log_saved_.exchange(true)) return; // 确保只存一次
        
        std::ofstream outfile("mpc_exec_times.csv");
        if (outfile.is_open())
        {
            outfile << "dt_us\n";
            for (auto t : mpc_exec_times_)
            {
                outfile << t << "\n";
            }
            outfile.close();
            RCLCPP_INFO(this->get_logger(), "✅ 成功将 %zu 条 MPC 耗时数据落盘到 mpc_exec_times.csv", mpc_exec_times_.size());
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "❌ 无法打开文件保存耗时数据！");
        }
    }


};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<interpolation>();
    rclcpp::spin(node);
    rclcpp::shutdown();    
    return 0;
}
