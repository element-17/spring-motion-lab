/*
 * 负责人为钟江泽。
 * TODO 1 从参数和当前采样读取质量 m、弹力系数 k、阻尼 c、位移 x 和速度 v。
 *        计算加速度 a = (-k * x - c * v) / m。
 * TODO 2 取 dt 为时间步长，先计算 vNext = v + a * dt，再计算 xNext = x + vNext * dt。
 * TODO 3 把新时间、位移和速度装入新的运动采样并返回。
 * 使用默认参数推进 1/120 秒，第一步速度约为 -0.02666667 米每秒，位移约为 0.19977778 米。
 */
package io.github.chenglei6arch.springmotionlab.simulation.engine;

import io.github.chenglei6arch.springmotionlab.simulation.model.MotionSample;
import io.github.chenglei6arch.springmotionlab.simulation.model.SpringParameters;

/**
 * 半隐式欧拉积分器。
 * 实现运动积分器接口。先计算新速度，再用新速度计算新位移，每次调用只计算一步。
 */
public final class SemiImplicitEulerIntegrator implements MotionIntegrator {
    /** 创建积分器。每次计算所需的数据由推进方法的参数提供。 */
    public SemiImplicitEulerIntegrator() {}

    /**
     * 按阻尼弹簧模型计算一个时间步。
     * 加速度等于弹簧力与阻尼力之和除以质量，其中弹簧力与位移反向，阻尼力与速度反向。
     *
     * @param parameters 提供弹力系数和摩擦系数的参数记录，质量由参数记录中的固定常量提供
     * @param current 上一步的时间、相对平衡位置的位移和速度
     * @param stepSeconds 本次推进的时长，单位为秒
     * @return 新的运动采样，时间增加一个步长，位移和速度更新为计算结果
     */
    @Override
    public MotionSample step(SpringParameters parameters, MotionSample current, double stepSeconds) {
        double mass = SpringParameters.MASS_KILOGRAMS;
        double stiffness = parameters.stiffnessNewtonsPerMeter();
        double damping = parameters.dampingNewtonSecondsPerMeter();
        double displacement = current.displacementMeters();
        double velocity = current.velocityMetersPerSecond();
        double dt = stepSeconds;

        double acceleration = (-stiffness * displacement - damping * velocity) / mass;
        double nextVelocity = velocity + acceleration * dt;
        double nextDisplacement = displacement + nextVelocity * dt;

        return new MotionSample(current.timeSeconds() + dt, nextDisplacement, nextVelocity);
    }
}
