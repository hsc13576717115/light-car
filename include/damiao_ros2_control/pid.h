#pragma once

#include <cmath>
#include <algorithm>

namespace damiao_ros2_control {

class PID {
public:
    struct Gains {
        double p = 0.0;  // 比例增益
        double i = 0.0;  // 积分增益
        double d = 0.0;  // 微分增益
    };

    struct Limits {
        double integral = -1.0;    // 积分限幅，-1表示无限制
        double output = -1.0;      // 输出限幅，-1表示无限制
        double derivative = -1.0;  // 微分限幅，-1表示无限制
    };

    PID() = default;
    
    explicit PID(double p, double i, double d) {
        setGains(p, i, d);
    }

    // 设置PID增益
    void setGains(double p, double i, double d) {
        gains_.p = p;
        gains_.i = i;
        gains_.d = d;
        reset();
    }

    void setGains(const Gains& gains) {
        gains_ = gains;
        reset();
    }

    // 设置限制参数
    void setLimits(double max_integral, double max_output, double max_derivative = -1.0) {
        limits_.integral = max_integral;
        limits_.output = max_output;
        limits_.derivative = max_derivative;
    }

    // 核心计算函数
    double compute(double error, double dt) {
        if (dt <= 0.0) {
            return 0.0;  // 防止除以零
        }

        // 计算积分项（带抗饱和）
        integral_ += error * dt;
        
        // 应用积分限幅
        if (limits_.integral > 0) {
            integral_ = std::clamp(integral_, -limits_.integral, limits_.integral);
        }

        // 计算微分项
        double derivative = (error - prev_error_) / dt;
        
        // 应用微分限幅
        if (limits_.derivative > 0) {
            derivative = std::clamp(derivative, -limits_.derivative, limits_.derivative);
        }

        // 计算原始输出
        double output = gains_.p * error + 
                       gains_.i * integral_ + 
                       gains_.d * derivative;

        // 应用输出限幅
        if (limits_.output > 0) {
            output = std::clamp(output, -limits_.output, limits_.output);
        }

        // 更新状态
        prev_error_ = error;
        prev_derivative_ = derivative;

        return output;
    }

    // 重置控制器状态
    void reset() {
        integral_ = 0.0;
        prev_error_ = 0.0;
        prev_derivative_ = 0.0;
    }

    // 获取当前增益
    const Gains& getGains() const { return gains_; }

    // 获取当前限制
    const Limits& getLimits() const { return limits_; }

private:
    Gains gains_;
    Limits limits_;
    double integral_ = 0.0;
    double prev_error_ = 0.0;
    double prev_derivative_ = 0.0;
};

} // namespace damiao_ros2_control