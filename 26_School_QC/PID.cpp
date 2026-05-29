#include "PID.hpp"

// 默认构造函数
PIDController::PIDController()
    : kp_(1.0), ki_(0.0), kd_(0.0),
      integral_(0.0), prev_error_(0.0), prev_output_(0.0),
      min_output_(-INFINITY), max_output_(INFINITY),
    min_integral_(-INFINITY), max_integral_(INFINITY),
    min_proportional_(-INFINITY), max_proportional_(INFINITY),
    min_derivative_(-INFINITY), max_derivative_(INFINITY),
    min_i_output_(-INFINITY), max_i_output_(INFINITY),
      last_time_(), dt_(0.001),
      first_run_(true), enable_integral_limit_(true),
      predictive_enabled_(false), predictive_horizon_(0.05), predictive_alpha_(0.5),
      max_integral_step_(0.5), max_i_output_step_(0.5),
      debug_enabled_(false), integral_target_(0.0), integral_target_active_(false) {}

// 带参数构造
PIDController::PIDController(double kp, double ki, double kd)
    : kp_(kp), ki_(ki), kd_(kd),
      integral_(0.0), prev_error_(0.0), prev_output_(0.0),
      min_output_(-INFINITY), max_output_(INFINITY),
    min_integral_(-INFINITY), max_integral_(INFINITY),
    min_proportional_(-INFINITY), max_proportional_(INFINITY),
    min_derivative_(-INFINITY), max_derivative_(INFINITY),
    min_i_output_(-INFINITY), max_i_output_(INFINITY),
      last_time_(), dt_(0.001),
      first_run_(true), enable_integral_limit_(true),
      predictive_enabled_(false), predictive_horizon_(0.05), predictive_alpha_(0.5),
      max_integral_step_(0.5), max_i_output_step_(0.5),
      debug_enabled_(false), integral_target_(0.0), integral_target_active_(false) {}

// 设置增益
void PIDController::setGains(double kp, double ki, double kd) {
    setKp(kp);
    setKi(ki);
    setKd(kd);
}

void PIDController::setKp(double kp) { kp_ = kp; }

// 平滑设置 Ki：将目标 integral 设为按比例缩放的值，然后平滑过渡
void PIDController::setKi(double ki) {
    // 直接设置 Ki（不做平滑或调整），恢复为裸 PID 行为
    ki_ = ki;
}

void PIDController::setKd(double kd) { kd_ = kd; }

void PIDController::enablePredictiveIntegral(bool enable) { predictive_enabled_ = enable; }
void PIDController::setPredictiveHorizon(double seconds) { if (seconds >= 0.0) predictive_horizon_ = seconds; }
void PIDController::setPredictiveAlpha(double alpha) { predictive_alpha_ = std::clamp(alpha, 0.0, 1.0); }
void PIDController::setMaxIntegralStep(double maxStep) { if (maxStep >= 0.0) max_integral_step_ = maxStep; }
void PIDController::setMaxIOutputStep(double maxStep) { if (maxStep >= 0.0) max_i_output_step_ = maxStep; }

void PIDController::setOutputLimits(double min, double max) {
    if (min > max) std::swap(min, max);
    min_output_ = min; max_output_ = max;
}

void PIDController::setProportionalLimits(double min, double max) {
    if (min > max) std::swap(min, max);
    min_proportional_ = min; max_proportional_ = max;
}

void PIDController::setDerivativeLimits(double min, double max) {
    if (min > max) std::swap(min, max);
    min_derivative_ = min; max_derivative_ = max;
}

void PIDController::setProportionalLimitsAndApply(double min, double max) {
    setProportionalLimits(min, max);
    // 无需同步内部状态，保留接口一致性
}

void PIDController::setDerivativeLimitsAndApply(double min, double max) {
    setDerivativeLimits(min, max);
    // 无需同步内部状态，保留接口一致性
}

void PIDController::setIOutputLimits(double min, double max) {
    if (min > max) std::swap(min, max);
    min_i_output_ = min; max_i_output_ = max;
    // 立即应用到当前 integral_
    if (ki_ != 0.0) {
        double iout = ki_ * integral_;
        if (iout < min_i_output_) iout = min_i_output_;
        else if (iout > max_i_output_) iout = max_i_output_;
        integral_ = iout / ki_;
    }
}

void PIDController::setIntegralLimits(double min, double max) {
    // 兼容历史接口：将传入的范围视为 I 输出（Iout）范围，并应用到当前 integral_
    if (min > max) std::swap(min, max);
    min_i_output_ = min; max_i_output_ = max;
    // 立即把当前 integral_ 调整为满足新的 Iout 限制
    if (ki_ != 0.0) {
        double iout = ki_ * integral_;
        if (iout < min_i_output_) iout = min_i_output_;
        else if (iout > max_i_output_) iout = max_i_output_;
        integral_ = iout / ki_;
    }
}

void PIDController::setIntegralLimitsAndApply(double min, double max) {
    min_i_output_ = min; max_i_output_ = max;
}

void PIDController::enableIntegralLimit(bool enable) { enable_integral_limit_ = enable; }
// enableDebug is defined inline in the header

void PIDController::reset() {
    integral_ = 0.0; prev_error_ = 0.0; prev_output_ = 0.0; first_run_ = true;
    integral_target_active_ = false;
}

// calculate: 主计算路径
double PIDController::calculate(double setpoint, double measurement) {
    double error = setpoint - measurement;
    // 比例项
    double proportional = kp_ * error;

    // 积分项（支持预测积分）
    double integral_out = 0.0;
    if (ki_ != 0.0) {
        // 裸 PID：简单积分累加，不做预测、不做单步限制、不做跨零减速、不做平滑
        double delta = error * dt_;
        integral_ += delta;
        if (enable_integral_limit_) integral_ = std::clamp(integral_, min_integral_, max_integral_);
        integral_out = ki_ * integral_;
        // 如果已设置 I 输出范围，则对 Iout 做夹限并同步 integral_
        if(integral_out <= min_i_output_) {
            integral_out = min_i_output_;
            if(ki_ != 0.0) {
                integral_ = integral_out / ki_;
            }
        }
        else if (integral_out >= max_i_output_) {
            integral_out = max_i_output_;
            if(ki_ != 0.0) {
                integral_ = integral_out / ki_;
            }
        }
    }

    // 微分项
    double derivative = 0.0;
    if (kd_ != 0.0 && !first_run_ && dt_ > 0.0) {
        derivative = kd_ * (error - prev_error_) / dt_;
    }

    // 对比例项和微分项分别做范围限定（如果已设）
    if (proportional < min_proportional_) proportional = min_proportional_;
    else if (proportional > max_proportional_) proportional = max_proportional_;

    if (derivative < min_derivative_) derivative = min_derivative_;
    else if (derivative > max_derivative_) derivative = max_derivative_;

    double output = proportional + integral_out + derivative;
    

    // 仅限幅输出，不进行抗饱和回退（裸 PID 行为）
    if (output > max_output_) {
        output = max_output_;
    } else if (output < min_output_) {
        output = min_output_;
    }
    if (debug_enabled_) {
        std::cout << "P:" << proportional << " I:" << integral_out << " D:" << derivative << " out:" << output << std::endl;
    }

    prev_error_ = error;
    prev_output_ = output;
    first_run_ = false;
    return output;
}

double PIDController::calculateAuto(double setpoint, double measurement) {
    auto now = std::chrono::steady_clock::now();
    if (first_run_) {
        last_time_ = now; dt_ = 0.001;
    } else {
        dt_ = std::chrono::duration<double>(now - last_time_).count();
        if (dt_ > 0.1) dt_ = 0.1;
    }
    last_time_ = now;
    return calculate(setpoint, measurement);
}

void PIDController::setIntegral(double integral) {
    integral_ = integral;
    if (enable_integral_limit_) integral_ = std::clamp(integral_, min_integral_, max_integral_);
}

void PIDController::setIntegralTarget(double target) {
    if (enable_integral_limit_) target = std::clamp(target, min_integral_, max_integral_);
    integral_target_ = target;
    integral_target_active_ = true;
}

double PIDController::PID_Interal_out(PIDController& pid) {
    return integral_;
}
