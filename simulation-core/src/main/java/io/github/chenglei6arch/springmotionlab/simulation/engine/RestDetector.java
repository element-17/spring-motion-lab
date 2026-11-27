/*
 * 负责人为钟江泽。
 * TODO 1 检查采样的位移绝对值和速度绝对值是否同时严格低于阈值。
 * TODO 2 两项都达标时给连续步数加一，任一项达到或超过阈值时清零。
 * TODO 3 连续达标步数达到规定值后，返回已静止。
 * 验收时检查经过平衡点、到达转折点和连续达标至规定时长三种情况。
 */
package io.github.chenglei6arch.springmotionlab.simulation.engine;

import io.github.chenglei6arch.springmotionlab.simulation.model.MotionSample;

/**
 * 静止检测器。
 * 每轮仿真使用一个检测器，记录位移和速度同时接近零的连续步数。
 * 连续达到规定的模拟时长才算静止，以免弹簧经过平衡点（位移为零，速度较大）时提前停下。
 */
final class RestDetector {
    /** 位移绝对值需要低于的阈值，单位为米。 */
    static final double DISPLACEMENT_TOLERANCE_METERS = 0.002;
    /** 速度绝对值需要低于的阈值，单位为米每秒。 */
    static final double SPEED_TOLERANCE_METERS_PER_SECOND = 0.005;
    /** 判定静止需要连续达标的模拟时长，单位为毫秒。 */
    static final int REST_DURATION_MILLIS = 400;
    /** 由达标时长和每秒步数换算得到的最少步数，向上取整。 */
    static final int REQUIRED_STEPS = Math.ceilDiv(REST_DURATION_MILLIS * SpringSimulator.STEPS_PER_SECOND, 1_000);

    /** 连续同时满足位移与速度阈值的步数，任一项不达标时归零。 */
    private int consecutiveSteps;

    /**
     * 观察一个采样并更新连续达标步数。该方法只更新计数，无返回值。
     *
     * @param sample 仿真器完成当前时间步后得到的运动采样
     */
    void observe(MotionSample sample) {
        boolean displacementCloseToZero = Math.abs(sample.displacementMeters()) < DISPLACEMENT_TOLERANCE_METERS;
        boolean speedCloseToZero = Math.abs(sample.velocityMetersPerSecond()) < SPEED_TOLERANCE_METERS_PER_SECOND;
        if (displacementCloseToZero && speedCloseToZero) {
            consecutiveSteps = consecutiveSteps + 1;
        } else {
            consecutiveSteps = 0;
        }
    }

    /**
     * 查询当前连续步数是否达到静止要求。查询本身不计步。
     *
     * @return 连续达标至少达到 {@code REQUIRED_STEPS} 步时返回 {@code true}，否则返回 {@code false}
     */
    boolean isSettled() {
        return consecutiveSteps >= REQUIRED_STEPS;
    }
}
