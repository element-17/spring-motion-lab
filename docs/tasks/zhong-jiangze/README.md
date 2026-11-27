# 钟江泽的开发任务

你负责 Java 的两件事。给定当前位移和速度，算出下一时刻的运动状态，再根据连续采样判断弹簧什么时候停下来。两部分可以分开写、分开验收。

## 开工

用 IntelliJ IDEA 打开 `spring-motion-lab`，也就是包含 `settings.gradle.kts` 和 `gradlew.bat` 的目录。将 Gradle JVM 设为 JDK 21，然后在项目终端切到自己的分支。

```powershell
git fetch origin
git switch "钟江泽"
git branch --show-current
```

最后一行显示 `钟江泽` 就可以开始。首次切换时，Git 会根据同名远程分支建立本地分支。

## 要改的文件在哪里

下表从仓库根目录开始。可以在 IDE 左侧的 **Project** 文件树逐级展开，也可以复制完整路径定位。点表里的路径可以直接在 GitHub 查看文件开头的 TODO。

| 顺序 | 完整路径 | 修改位置 |
|---|---|---|
| 1 | [`simulation-core/src/main/java/io/github/chenglei6arch/springmotionlab/simulation/engine/SemiImplicitEulerIntegrator.java`](../../../simulation-core/src/main/java/io/github/chenglei6arch/springmotionlab/simulation/engine/SemiImplicitEulerIntegrator.java) | `step(parameters, current, stepSeconds)` |
| 2 | [`simulation-core/src/main/java/io/github/chenglei6arch/springmotionlab/simulation/engine/RestDetector.java`](../../../simulation-core/src/main/java/io/github/chenglei6arch/springmotionlab/simulation/engine/RestDetector.java) | 连续计数、`observe(sample)`、`isSettled()` |

## 1. 实现单步运动

1. 在 `step()` 内从 `current` 取位移 `x`、速度 `v` 和时间。用 `SpringParameters` 取弹力系数 `k` 和阻尼 `c`，用已有常量 `MASS_KILOGRAMS` 取质量 `m`。
2. 计算 `a = (-k * x - c * v) / m`。用传进来的 `stepSeconds` 作 `dt`，先算 `vNext = v + a * dt`，然后用新速度算 `xNext = x + vNext * dt`。
3. 把 `current.timeSeconds() + dt`、`xNext`、`vNext` 装进一个新的 `MotionSample` 并返回。这个方法每次只算一步。

用默认参数核对前两步。质量为 1 kg，`k = 16`，`c = 2`，初始状态为 `t = 0`、`x = 0.20`、`v = 0`，`dt = 1/120 s`。

| 步数 | 时间（s） | 位移（m） | 速度（m/s） |
|---:|---:|---:|---:|
| 1 | 0.00833333 | 0.19977778 | -0.02666667 |
| 2 | 0.01666667 | 0.19933728 | -0.05285926 |

## 2. 实现静止判断

1. 在 `RestDetector` 加一个实例字段，记录连续满足条件的步数，初值为 0。
2. `observe(sample)` 每次只看当前采样。位移绝对值严格小于 `0.002 m`，且速度绝对值严格小于 `0.005 m/s` 时计数加 1。任一项达到或超过阈值时计数归零。代码直接使用文件里已有的阈值常量。
3. `isSettled()` 在计数达到 `REQUIRED_STEPS` 后返回 `true`。这个常量由持续时长和每秒步数换算，当前算出 48 步。复位由控制器创建新的模拟器，静止检测器也随之重新初始化。

可以用 `x = 0`、`v = 0.1` 代表经过平衡点，用 `x = 0.1`、`v = 0` 代表经过转折点，两种情况都应清零。连续喂 `REQUIRED_STEPS - 1` 次 `x = 0.001`、`v = 0.003` 时应返回 `false`，再喂一次才返回 `true`。

已有的 [`simulation-core/src/main/java/io/github/chenglei6arch/springmotionlab/simulation/engine/SpringSimulator.java`](../../../simulation-core/src/main/java/io/github/chenglei6arch/springmotionlab/simulation/engine/SpringSimulator.java) 负责按固定时间步调用积分器和检测器。[`simulation-core/src/main/java/io/github/chenglei6arch/springmotionlab/simulation/model/MotionSample.java`](../../../simulation-core/src/main/java/io/github/chenglei6arch/springmotionlab/simulation/model/MotionSample.java) 定义了返回数据。[架构和公式](../../architecture.md) · [接口文档](../../api.md)

## 怎么验收

```powershell
.\gradlew.bat :simulation-core:test
```

首两步可在 IDE 调试或临时运行直接调用 `step()` 核对。公开测试补齐后，再与阿卜杜热合曼核对静止判定用例。提交前检查默认参数长时间运行时振幅整体衰减，经过平衡点时弹簧会继续运动。
