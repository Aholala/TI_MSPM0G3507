# provincial_firstprize

STM32F103RCT6 自动行驶小车工程。当前代码已经完成底盘、电机、编码器、灰度循迹、按键、OLED、蜂鸣器和 LED 的基础驱动，并提供题号/圈数设置界面。

注意：题目场地只有左右两个半圆弧是黑线，直线段没有黑线。因此“全程循迹”是不对的。当前代码的循迹控制只能用于黑色半圆弧段；A-B、C-D、A-C、B-D 这些无线段后续需要用编码器里程和 IMU 航向做路径状态机。

## 硬件映射

所有主要板级参数集中在 `User/Bsp/board_config.h`。

当前引脚：

| 功能 | 引脚 | 说明 |
| --- | --- | --- |
| K1 | PA8 | 题号切换，低电平有效 |
| K2 | PC13 | 圈数切换，低电平有效 |
| K3 | PC14 | 启动/停止，低电平有效 |
| 蜂鸣器 | PB5 | 高电平有效 |
| LED | PC4 | 默认高电平有效 |
| TB6612 左电机方向 | PA5 / PA4 | IN1 / IN2 |
| TB6612 右电机方向 | PB0 / PB1 | IN1 / IN2 |
| TB6612 PWM | TIM3 CH1 / CH2 | 左 / 右 |
| 编码器 | TIM4 / TIM8 | 左 / 右 |
| 灰度传感器 | PC0 PC1 PC2 PC3 PC5 PC8 PC9 PC12 | 8 路 |

灰度模块配置为黑线高电平：

```c
#define BOARD_LINE_SENSOR_DEFAULT_ACTIVE_LOW 0u
```

如果你的 LED 是低电平点亮，修改：

```c
#define BOARD_LED_DEFAULT_ACTIVE_LOW 1u
```

## 需要重点调的参数

### 1. 编码器每圈脉冲数

文件：`User/Bsp/board_config.h`

```c
#define BOARD_ENCODER_OUTPUT_PULSES_PER_REV 364u
```

你给出的参数是 28 减速比电机，输出轴一圈 364 个脉冲。建议实测一次：手动转输出轴一圈，看 `BspEncoder_GetTotal()` 增加多少。

如果实测约 364，保持不变。  
如果实测约 728 或 1456，把这个宏改成实测值。

### 2. 左右编码器方向

文件：`User/Bsp/board_config.h`

```c
#define BOARD_ENCODER_LEFT_INVERTED  0u
#define BOARD_ENCODER_RIGHT_INVERTED 0u
```

调试方法：让车向前走，左右编码器增量都应该是正数。如果某一侧为负，把对应宏改成 `1u`。

### 3. 轮径

文件：`User/Bsp/board_config.h`

```c
#define BOARD_WHEEL_DIAMETER_MM 65u
```

当前轮径为 65mm。代码会据此计算编码器脉冲对应的行驶距离，用于后续直线段里程控制。

### 4. 默认巡线速度

文件：`User/Bsp/board_config.h`

```c
#define BOARD_CHASSIS_DEFAULT_SPEED_RPM 300u
```

这个值会换算成每个控制周期的编码器目标增量。当前控制周期是 10ms：

```c
#define BOARD_CONTROL_TASK_PERIOD_MS 10u
```

如果车太快、容易冲出线，先降到 `150u` 或 `200u`。如果太慢，再逐步加。

### 5. 循迹转向比例

文件：`User/App/task/control_task.c`

```c
#define CONTROL_TASK_LINE_ERROR_DIV 20
```

这个值越小，转向越猛；越大，转向越柔。  
调试建议：

- 直线轻微摆动：适当增大，例如 `25`、`30`
- 过弯转不过来：适当减小，例如 `15`
- 大幅蛇形：先降低速度，再增大这个值

### 6. 灰度传感器权重

文件：`User/Module/module_tracker.c`

```c
static const int16_t sensor_weights[LINE_TRACKER_SENSOR_COUNT] = {
    -3500, -2500, -1500, -500, 500, 1500, 2500, 3500
};
```

如果灰度传感器物理顺序和代码顺序相反，车会越修越偏。可以交换 `bsp_line_sensor.c` 里的引脚顺序，或者反转权重顺序。

### 7. 速度 PID

文件：`User/Bsp/board_config.h`

```c
#define BOARD_CHASSIS_SPEED_PID_KP 30
#define BOARD_CHASSIS_SPEED_PID_KI 2
#define BOARD_CHASSIS_SPEED_PID_KD 0
#define BOARD_CHASSIS_SPEED_PID_SCALE 1
```

当前速度 PID 的目标值和反馈值单位都是“每 10ms 编码器脉冲数”。例如默认 300RPM、364 脉冲/圈时，目标速度约为 18 脉冲/10ms。

调试顺序：

1. 先设 `KI=0`、`KD=0`，只调 `KP`。
2. 让车悬空，给一个较小速度，例如 `g_control_task_base_speed = 10`。
3. 看 `g_app_chassis_left_speed_pid.feedback` 是否能接近 `target`。
4. 如果反馈上不去，增大 `KP`。
5. 如果输出和速度振荡，减小 `KP`。
6. `KP` 基本稳定后，再少量增加 `KI`，用来消除长期误差。
7. 如果加 `KI` 后容易冲、积分项很大，缩小 `KI` 或收紧积分限幅。

不建议一开始加 `KD`。编码器速度是离散脉冲，微分项容易放大抖动。

### 8. 按键消抖和长按

文件：`User/Bsp/board_config.h`

```c
#define BOARD_KEY_DEFAULT_DEBOUNCE_MS   20u
#define BOARD_KEY_DEFAULT_LONG_PRESS_MS 1000u
```

如果按键触发不稳，增大消抖时间，例如 `30u` 或 `50u`。

### 9. 声光提示时间

文件：`User/App/app_race_config.c`

```c
#define APP_RACE_NOTIFY_KEY_MS    60u
#define APP_RACE_NOTIFY_POINT_MS  150u
#define APP_RACE_NOTIFY_FINISH_MS 300u
```

分别对应按键提示、经过 A/B/C/D 点提示、完成提示。

## 代码架构

项目按三层组织：

### Module 层

目录：`User/Module`

这一层尽量不依赖 STM32 HAL，方便后期移植。

| 文件 | 作用 |
| --- | --- |
| `module_motor.*` | 通用直流电机抽象，负责正反转、刹车、速度限幅 |
| `module_pid.*` | PID 控制器 |
| `module_tracker.*` | 8 路灰度循迹算法，计算黑线位置和误差 |
| `module_key.*` | 非阻塞按键消抖，支持按下/释放/长按事件 |
| `module_buzzer.*` | 蜂鸣器开关抽象 |
| `module_led.*` | LED 开关抽象 |
| `module_icm45686.*`、`inv_imu_*` | ICM45686 IMU 驱动/封装 |

### BSP 层

目录：`User/Bsp`

这一层知道具体芯片、GPIO、TIM、I2C、SPI。

| 文件 | 作用 |
| --- | --- |
| `board_config.h` | 板级引脚、有效电平、轮径、编码器参数集中配置 |
| `bsp_tb6612.*` | TB6612 电机驱动板适配 |
| `bsp_encoder.*` | TIM 编码器读取、累计脉冲、脉冲转距离 |
| `bsp_line_sensor.*` | 8 路灰度传感器 GPIO 读取 |
| `bsp_key.*` | PA8/PC13/PC14 三个按键 |
| `bsp_buzzer.*` | PB5 蜂鸣器 |
| `bsp_led.*` | PC4 LED |
| `bsp_oled.*`、`bsp_oled_data.*` | OLED 显示 |
| `bsp_icm45686.*` | IMU 的 SPI/中断适配 |

移植到其他芯片时，优先改 `board_config.h` 和 `User/Bsp`，尽量不动 `User/Module`。

### App 层

目录：`User/App`

这一层组合 BSP 和 Module，形成小车功能。

| 文件 | 作用 |
| --- | --- |
| `app_chassis.*` | 底盘应用，绑定电机和编码器，提供目标速度、速度闭环、快照 |
| `app_line_follow.*` | 灰度循迹应用，周期读取传感器并保存误差 |
| `app_race_config.*` | 题号/圈数设置、OLED 显示、启停状态、声光提示 |
| `app_icm45686.*` | IMU 应用封装 |
| `icm45686_port.*` | IMU 移植接口 |

任务入口在 `User/App/task`：

| 文件 | 周期 | 作用 |
| --- | --- | --- |
| `line_follow_task.c` | 5ms | 更新灰度循迹误差 |
| `chassis_task.c` | 10ms | 更新编码器并运行速度 PID |
| `control_task.c` | 10ms | 根据循迹误差给左右轮目标速度 |
| `imu_task.c` | 由文件内部决定 | 更新 IMU |
| `freertos.c` 的 defaultTask | 20ms | 运行按键/OLED/声光提示配置界面 |

## 当前运行逻辑

上电后：

1. `main.c` 初始化 GPIO、DMA、TIM、I2C、SPI。
2. 初始化底盘、循迹、比赛配置界面。
3. FreeRTOS 启动任务。
4. OLED 显示题号、目标圈数、当前圈数、状态和路径。
5. PA8 切换题号 Q1-Q4。
6. PC13 切换目标圈数 1-4。
7. PC14 切换启动/停止。
8. 未启动时，控制任务给底盘目标速度 0。
9. 启动后，控制任务按灰度误差做基础循迹。

当前 `control_task.c` 是基础循迹控制：

```c
turn = line_error / CONTROL_TASK_LINE_ERROR_DIV;
left_target  = base_speed - turn;
right_target = base_speed + turn;
```

这个逻辑只适合有黑线的半圆弧段。

## 题目路径拆分

场地只有两个半圆弧是黑线，直线/斜线段没有黑线。

| 题目 | 路径 | 控制方式 |
| --- | --- | --- |
| Q1 | A -> B | 编码器/IMU 直行 100cm，到 B 停车 |
| Q2 | A -> B -> C -> D -> A | A-B 里程直行，B-C 右半圆循迹，C-D 里程直行，D-A 左半圆循迹 |
| Q3 | A -> C -> B -> D -> A | A-C 定向行驶，C-B 右半圆反向循迹，B-D 定向行驶，D-A 左半圆循迹 |
| Q4 | 按 Q3 跑 4 圈 | 同 Q3，圈数为 4 |

后续应该新增 `app_race_runner.*`：

- 根据 `AppRaceConfig_GetMode()` 生成路径段
- 根据 `AppRaceConfig_GetTargetLaps()` 判断完成圈数
- 直线/斜线段用编码器距离和 IMU 航向
- 半圆弧段用灰度循迹
- 每到 A/B/C/D 调用 `AppRaceConfig_NotifyPoint(now_ms)`
- 结束时调用 `AppRaceConfig_SetFinished()`

## 建议调试顺序

1. 单独测试按键、OLED、蜂鸣器、LED。
2. 测试电机方向：前进时左右轮方向一致。
3. 测试编码器方向：前进时左右编码器增量都为正。
4. 手转输出轴一圈，校准 `BOARD_ENCODER_OUTPUT_PULSES_PER_REV`。
5. 推车或慢速跑 100cm，校准轮径/距离换算。
6. 测试灰度传感器黑线高电平是否正确。
7. 低速测试半圆循迹，调 `BOARD_CHASSIS_DEFAULT_SPEED_RPM` 和 `CONTROL_TASK_LINE_ERROR_DIV`。
8. 再实现并调试比赛路径状态机。

## Ozone 调试建议

推荐在 Ozone 的 Watch 窗口里直接观察这些全局变量。它们是专门给调试暴露的快照，不需要在 Watch 里调用函数。

### 比赛设置状态

变量：

```c
g_debug_race_config_snapshot
```

结构体类型：`AppRaceConfig_Snapshot`

重点看：

| 字段 | 含义 |
| --- | --- |
| `mode` | 当前题号，1-4 |
| `target_laps` | 目标圈数 |
| `current_lap` | 当前圈数 |
| `state` | 状态，0=IDLE，1=READY，2=RUNNING，3=FINISHED |

按 PA8、PC13、PC14 时，先看这个结构体是否变化。如果这里不变，优先查按键 GPIO 和按键有效电平。

### 循迹状态

变量：

```c
g_debug_line_follow_snapshot
```

结构体类型：`AppLineFollow_Snapshot`

重点看：

| 字段 | 含义 |
| --- | --- |
| `raw_mask` | 原始 8 路灰度输入，bit0-bit7 对应传感器 0-7 |
| `active_mask` | 根据有效电平转换后的黑线 mask |
| `position` | 黑线位置，负数偏左，正数偏右 |
| `error` | 当前给控制使用的循迹误差 |
| `status` | 0=正常，1=丢线，2=全检测到 |

黑线高电平时，压到黑线的传感器在 `raw_mask` 里应该为 1。  
如果 `raw_mask` 对，但 `active_mask` 反了，改 `BOARD_LINE_SENSOR_DEFAULT_ACTIVE_LOW`。

### 底盘和编码器状态

变量：

```c
g_debug_chassis_snapshot
```

结构体类型：`AppChassis_Snapshot`

重点看：

| 字段 | 含义 |
| --- | --- |
| `left_target_speed` / `right_target_speed` | 左右轮目标速度，单位是每控制周期编码器脉冲 |
| `left_encoder_delta` / `right_encoder_delta` | 左右轮本周期编码器增量 |
| `left_encoder_total` / `right_encoder_total` | 左右轮累计编码器脉冲 |
| `left_encoder_mrev` / `right_encoder_mrev` | 左右轮累计转数，单位 0.001 圈 |
| `left_encoder_mm` / `right_encoder_mm` | 左右轮累计距离，单位 mm |
| `left_control_output` / `right_control_output` | 速度 PID 输出到电机的值 |

调电机方向时，让车向前，`left_encoder_delta` 和 `right_encoder_delta` 应该都为正。  
调 100cm 直线时，看 `left_encoder_mm` 和 `right_encoder_mm` 是否接近 1000。

### 控制任务变量

变量：

```c
g_control_task_base_speed
g_control_task_turn
```

说明：

| 变量 | 含义 |
| --- | --- |
| `g_control_task_base_speed` | 手动覆盖基础速度。为 0 时使用 `BOARD_CHASSIS_DEFAULT_SPEED_DELTA` |
| `g_control_task_turn` | 当前循迹转向量 |

调试时可以在 Ozone 里直接改 `g_control_task_base_speed`，例如先写 `10`、`15`、`20`，观察车速。不要一开始给太大。

### PID 参数

变量：

```c
g_app_chassis_left_speed_pid
g_app_chassis_right_speed_pid
```

结构体类型：`PidController`

重点看：

| 字段 | 含义 | 调参判断 |
| --- | --- | --- |
| `cfg.kp` | 比例系数 | 主要决定响应力度 |
| `cfg.ki` | 积分系数 | 消除长期速度误差 |
| `cfg.kd` | 微分系数 | 一般先不用 |
| `cfg.scale` | 输出缩放 | 输出 = 原始 PID / scale |
| `target` | 目标速度 | 每控制周期编码器脉冲 |
| `feedback` | 实际速度 | 每控制周期编码器脉冲 |
| `error` | `target - feedback` | 越接近 0 越好 |
| `integral` | 误差积分 | 长期顶到限幅说明积分过强或阻力大 |
| `derivative` | 误差变化量 | 抖动大时不要加 KD |
| `output` | PID 输出 | 最终给电机的 PWM 等效速度 |

如果目标速度有值，但 `encoder_delta` 长期为 0，查编码器和电机。  
如果 `encoder_delta` 有值但 `control_output` 过大或振荡，调 PID 参数。

建议 Ozone 里同时看：

```c
g_debug_chassis_snapshot.left_target_speed
g_debug_chassis_snapshot.left_encoder_delta
g_debug_chassis_snapshot.left_control_output
g_app_chassis_left_speed_pid
```

右轮同理。调车时先左右轮分开看，保证同一个目标速度下左右反馈接近。

### Kalman / IMU 参数

变量：

```c
g_debug_icm45686_snapshot
g_debug_attitude_estimator
```

结构体类型：

- `app_icm45686_snapshot_t`
- `imu_attitude_estimator_t`

先看 `g_debug_icm45686_snapshot`：

| 字段 | 含义 |
| --- | --- |
| `initialized` | IMU 是否初始化成功 |
| `data_ready` | 是否有新数据 |
| `last_init_status` | 初始化结果，0 为成功 |
| `last_process_status` | 最近一次处理结果，0 为成功 |
| `update_count` | 姿态更新次数，应持续增加 |
| `solution.raw` | 原始加速度/角速度 |
| `solution.filtered` | Kalman 1D 滤波后的加速度/角速度 |
| `solution.euler.roll_deg/pitch_deg/yaw_deg` | 当前欧拉角 |
| `solution.compensation.gyro_bias_radps` | 陀螺仪零偏估计 |
| `solution.compensation.stationary` | 是否被判断为静止 |
| `solution.compensation.accel_trust` | 加速度可信度 |

再看 `g_debug_attitude_estimator`：

| 字段 | 含义 | 调参影响 |
| --- | --- | --- |
| `sensor_filter.accel[i].q` | 加速度 1D Kalman 过程噪声 | 越大越跟手，越小越平滑 |
| `sensor_filter.accel[i].r` | 加速度测量噪声 | 越大越不信测量，越平滑但延迟大 |
| `sensor_filter.gyro[i].q` | 陀螺仪过程噪声 | 越大越跟手 |
| `sensor_filter.gyro[i].r` | 陀螺仪测量噪声 | 越大越平滑 |
| `roll_filter.q_angle` / `pitch_filter.q_angle` | 角度过程噪声 | 越大角度响应越快 |
| `roll_filter.q_bias` / `pitch_filter.q_bias` | 零偏估计速度 | 越大零偏修正越快，也更容易抖 |
| `roll_filter.r_measure` / `pitch_filter.r_measure` | 角度测量噪声 | 越大越相信陀螺积分，越小越相信加速度角 |
| `gyro_bias_radps` | 当前估计零偏 | 静止时应逐渐接近真实零偏 |
| `yaw_rad` | 航向角积分 | 当前 yaw 来源主要是陀螺仪积分 |

当前默认值在 `User/Lib/lib_kalman.c` 和 `User/Lib/lib_attitude.c`：

```c
kalman_1d_init(&filter->accel[i], 0.05f, 4.0f, 0.0f);
kalman_1d_init(&filter->gyro[i], 0.01f, 0.5f, 0.0f);
kalman_angle_init(&estimator->roll_filter, 0.001f, 0.003f, ...);
kalman_angle_init(&estimator->pitch_filter, 0.001f, 0.003f, ...);
```

Kalman 调试建议：

1. 先把车静止放平，看 `roll_deg/pitch_deg` 是否接近 0，`yaw_deg` 是否缓慢漂移。
2. 静止时如果角度抖动大，增大 `r` 或减小 `q`。
3. 动起来时如果角度反应太慢，增大 `q` 或减小 `r`。
4. `yaw_deg` 没有磁力计校正，长时间一定会漂。比赛短时间定向可以用，但要在启动前静止几秒让 `gyro_bias_radps.z` 收敛。
5. 如果 `stationary` 静止时一直不是 1，调大 `stationary_gyro_threshold_radps` 或 `stationary_accel_tolerance_mps2`。
6. 如果运动时被误判静止，调小这两个阈值。

用于直线/斜线行驶时，优先用 `solution.euler.yaw_deg` 控方向，用编码器 `left_encoder_mm/right_encoder_mm` 控距离。

### 常用断点位置

建议断点：

| 文件 | 函数 | 用途 |
| --- | --- | --- |
| `User/App/app_race_config.c` | `AppRaceConfig_RunOnce` | 看按键是否触发 |
| `User/App/app_line_follow.c` | `AppLineFollow_RunOnce` | 看灰度数据是否刷新 |
| `User/App/task/control_task.c` | `StartcontrolTask` 循环内部 | 看目标速度和转向量 |
| `User/App/app_chassis.c` | `AppChassis_UpdateEncoder` | 看编码器增量 |
| `User/App/app_chassis.c` | `AppChassis_SpeedControlRun` | 看 PID 输出 |

### Ozone 中建议先看的顺序

1. `g_debug_race_config_snapshot.state` 是否从 READY 变 RUNNING。
2. `g_debug_line_follow_snapshot.raw_mask` 是否随黑线变化。
3. `g_debug_line_follow_snapshot.error` 是否方向正确。
4. `g_debug_chassis_snapshot.left_target_speed/right_target_speed` 是否非 0。
5. `g_debug_chassis_snapshot.left_encoder_delta/right_encoder_delta` 是否为正。
6. `g_debug_chassis_snapshot.left_encoder_mm/right_encoder_mm` 是否和实际距离接近。

## 构建

```powershell
cmake --build build\Debug
```

当前工程已能通过上述命令编译。
