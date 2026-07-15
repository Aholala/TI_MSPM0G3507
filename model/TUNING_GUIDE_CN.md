# MSPM0 小车调参与编译

## 一键编译

双击工程根目录的 `build_firmware.bat`。

看到 `BUILD OK` 后，只烧录下面这一个文件：

`D:\AAA_MSPM0\Project\model\Debug\model.out`

## 调参顺序

必须先调左右轮速度环，再调循迹环。调速度环时把车轮架空，确认安全。

### 1. 进入纯 300 RPM 测速模式

打开 `User/Bsp/board_config.h`，修改：

```c
#define BOARD_DIRECT_300_RPM_TEST 1u
```

双击编译脚本并烧录。按 RUN 后左右轮固定目标均为 300 RPM。

### 2. 在线调速度环

在调试器 Watch 中添加：

```text
g_left_speed_feedforward_at_rated
g_right_speed_feedforward_at_rated
g_left_speed_pid_kp_x100
g_left_speed_pid_ki_x100
g_left_speed_pid_kd_x100
g_right_speed_pid_kp_x100
g_right_speed_pid_ki_x100
g_right_speed_pid_kd_x100
g_debug_chassis_snapshot
```

参数单位：`150` 表示 1.50，`25` 表示 0.25。

建议步骤：

1. `Kp=100, Ki=0, Kd=0`，前馈先设为 `900`。
2. 每次把 Kp 增加 25，直到响应足够快但不会来回波动；振荡后退约 20%。
3. Ki 从 5 开始，每次增加 5，直到两轮能稳定在 300 RPM且静差消失。
4. 编码器每 10 ms 的计数有量化跳动，Kd 默认保持0。
5. 若 `control_output=1000` 而实际仍低于300 RPM，这是供电或负载极限，不是PID问题。

主要观察值：

```text
left_target_speed/right_target_speed：300 RPM时应为75
left_encoder_delta/right_encoder_delta：稳定时应在75附近
left_actual_rpm/right_actual_rpm：瞬时转速
left_actual_rpm_filtered/right_actual_rpm_filtered：慢速显示值
left_control_output/right_control_output：PWM，范围0~1000
```

在线调出的最终值需要写回 `User/Bsp/board_config.h`，否则重新上电会恢复默认值：

```c
BOARD_CHASSIS_LEFT_SPEED_PID_KP / BOARD_CHASSIS_RIGHT_SPEED_PID_KP
BOARD_CHASSIS_LEFT_SPEED_PID_KI / BOARD_CHASSIS_RIGHT_SPEED_PID_KI
BOARD_CHASSIS_LEFT_SPEED_PID_KD / BOARD_CHASSIS_RIGHT_SPEED_PID_KD
BOARD_CHASSIS_LEFT_SPEED_FEEDFORWARD_AT_RATED
BOARD_CHASSIS_RIGHT_SPEED_FEEDFORWARD_AT_RATED
```

### 3. 恢复赛题并调循迹

速度环调完后改回：

```c
#define BOARD_DIRECT_300_RPM_TEST 0u
```

重新编译、烧录，然后在 Watch 中添加：

```text
g_line_follow_kp_x1000
g_line_follow_kd_x1000
g_line_follow_steering_sign
g_control_task_base_speed
g_control_task_turn
g_debug_line_follow_snapshot
```

建议循迹步骤：

1. `g_control_task_base_speed=40`，先用约160 RPM低速调方向。
2. 当前 `steering_sign=-1`。黑线在左边时车必须向左修；方向相反就改为1。
3. Kp从2开始逐步增加。转向不足就增加，左右摇头就降低。
4. Kd先保持0；若Kp合适但入弯仍有明显过冲，再从1开始尝试Kd。
5. 低速稳定后，把 `g_control_task_base_speed` 逐步改成50、60、75。
6. `g_control_task_base_speed=0` 表示使用默认300 RPM。

在线调出的循迹值需要写回 `User/App/task/control_task.c` 中对应全局变量的初始值。

## 常用换算

编码器为1500计数/圈，控制周期为10 ms：

```text
目标计数 = RPM / 4
160 RPM = 40
200 RPM = 50
240 RPM = 60
300 RPM = 75
```

第一问使用独立变量 `g_q1_cruise_rpm`，可以在Watch中直接填写RPM；
默认值为200，程序会限制在1~300 RPM，而且不会改变第二问速度。

## 第一问航向串级PID

MPU6050外环只接入第一问直线段，左右轮速度PID仍是内环。上电后先保持静止约4秒，
确认 `g_debug_mpu6050_snapshot.gyro_calibrated=1` 再启动。

Watch变量：

```text
g_q1_heading_pid_enable
g_q1_heading_pid_kp_x1000
g_q1_heading_pid_ki_x1000
g_q1_heading_pid_kd_x1000
g_q1_heading_steering_sign
g_q1_heading_max_turn_delta
g_q1_heading_target_deg
g_q1_heading_current_deg
g_q1_heading_error_deg
g_q1_heading_integral_deg_s
g_q1_heading_output_delta
```

先保持 `Ki=0, Kd=0`，Kp从300~500开始。偏航修不回来就逐步增加Kp；越修越偏先把
`g_q1_heading_steering_sign` 从-1改成1；左右摆动就降低Kp。方向和Kp正确后，再从很小的
Ki或Kd开始。初始最大修正建议保持4个编码器计数。
