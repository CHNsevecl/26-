#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include <chrono>
#include <cmath>
#include <iostream>
#include <algorithm>

class PIDController {
private:
    // PID参数
    double kp_;      // 比例增益
    double ki_;      // 积分增益
    double kd_;      // 微分增益
    
    // PID状态变量
    double integral_;      // 积分项
    double prev_error_;    // 上一次误差
    double prev_output_;   // 上一次输出
    
    // 限制参数
    double min_output_;    // 输出最小值
    double max_output_;    // 输出最大值
    double min_integral_;  // 积分项最小值
    double max_integral_;  // 积分项最大值
    double min_proportional_; // 比例项输出最小值
    double max_proportional_; // 比例项输出最大值
    double min_derivative_;   // 微分项输出最小值
    double max_derivative_;   // 微分项输出最大值
    double min_i_output_;     // I 输出最小值
    double max_i_output_;     // I 输出最大值
    
    // 时间相关
    std::chrono::steady_clock::time_point last_time_;
    double dt_;            // 时间差
    
    // 控制选项
    bool first_run_;       // 是否第一次运行
    bool enable_integral_limit_;  // 是否启用积分限制
    // 预测积分
    bool predictive_enabled_;    // 是否启用预测积分
    double predictive_horizon_;  // 预测时长（秒）
    double predictive_alpha_;    // 预测权重（0..1）
    double max_integral_step_;   // 单步积分最大变化量（抗突跃）
    double max_i_output_step_;    // 单步 I 输出最大变化量
    double integral_target_;       // 用于平滑外部设置的目标积分值
    bool integral_target_active_;  // 目标是否生效
    
    bool debug_enabled_ = false; // 是否启用调试输出
public:
    // 构造函数
    PIDController();
    PIDController(double kp, double ki, double kd);
    
    // 设置PID参数
    void setGains(double kp, double ki, double kd);
    void setKp(double kp);
    void setKi(double ki);
    void setKd(double kd);
    
    // 获取PID参数
    double getKp() const { return kp_; }
    double getKi() const { return ki_; }
    double getKd() const { return kd_; }
    
    // 设置输出限制
    void setOutputLimits(double min, double max);
    void setIntegralLimits(double min, double max);
    void setProportionalLimits(double min, double max);
    void setDerivativeLimits(double min, double max);
    void setIOutputLimits(double min, double max);
    // 便捷接口：设置并立即应用 P/D 限制（对 P/D 没有内部状态需要同步，提供一致的 API）
    void setProportionalLimitsAndApply(double min, double max);
    void setDerivativeLimitsAndApply(double min, double max);
    // 立即设置并应用积分范围（便捷接口）
    void setIntegralLimitsAndApply(double min, double max);
    
    // 设置积分限制使能
    void enableIntegralLimit(bool enable);
    // 预测积分设置
    void enablePredictiveIntegral(bool enable);
    void setPredictiveHorizon(double seconds);
    void setPredictiveAlpha(double alpha);
    void setMaxIntegralStep(double maxStep);
    void setMaxIOutputStep(double maxStep);
    // 平滑积分目标
    void setIntegralTarget(double target);

    void enableDebug(bool enable) {
        debug_enabled_ = enable;
    }
    
    // 重置PID状态
    void reset();
    
    // 计算PID输出（基于固定时间步长）
    double calculate(double setpoint, double measurement);
    
    // 计算PID输出（自动计算时间步长）
    double calculateAuto(double setpoint, double measurement);
    
    // 获取当前状态
    double getIntegral() const { return integral_; }
    double getPrevError() const { return prev_error_; }
    
    // 设置积分项（用于手动干预）
    void setIntegral(double integral);

    // 获取当前 I 输出（仅供调试）
    double PID_Interal_out(PIDController& pid);
};

#endif // PIDCONTROLLER_H