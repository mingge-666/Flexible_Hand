#ifndef FINGER_CONFIG_H
#define FINGER_CONFIG_H

#define FINGER_CMD_TASK_PRIO    5
#define FINGER_CMD_TASK_STACK   4096
#define FINGER_CMD_LINE_LEN     32

/* 控制任务：替代软件定时器，避免在定时器回调里做阻塞 I2C */
#define FINGER_CTRL_TASK_PRIO   7
#define FINGER_CTRL_TASK_STACK  4096

#define FINGER_UPDATE_PERIOD_MS 5
#define FINGER_UPDATE_DT        (FINGER_UPDATE_PERIOD_MS / 1000.0f)

/* ADC 采样与打印周期（统一在此调整） */
#define FINGER_ADC_SAMPLE_PERIOD_MS  5
#define FINGER_ADC_WINDOW_SIZE       10
#define FINGER_PRINT_PERIOD_MS       100

/* 运动参数默认值（可调） */

//加速度：
#define FINGER_DANGER_ACCEL_DPS2  150.0f  //加速度：150°/s²，危险模式，速度慢
#define FINGER_SLOW_ACCEL_DPS2    250.0f  //加速度：250°/s²，慢速模式
#define FINGER_DEFAULT_ACCEL_DPS2 350.0f  //加速度：350°/s²，默认模式
#define FINGER_MIDDLE_ACCEL_DPS2  500.0f  //加速度：500°/s²，中速模式
#define FINGER_FAST_ACCEL_DPS2    900.0f  //加速度：900°/s²，快速模式


//速度：
#define FINGER_DANGER_MAX_SPEED   50.0f  //最大速度：50°/s，危险模式，速度慢
#define FINGER_SLOW_MAX_SPEED     80.0f  //最大速度：80°/s，慢速模式
#define FINGER_DEFAULT_MAX_SPEED  140.0f //最大速度：140°/s，默认模式
#define FINGER_MIDDLE_MAX_SPEED   200.0f //最大速度：200°/s，中速模式
#define FINGER_FAST_MAX_SPEED     280.0f //最大速度：280°/s，快速模式


#define FINGER_DEFAULT_MIN_SPEED  5.0f   //最小速度：5°/s，防止太慢

/* 接触判定：压力读数超过该阈值即认为已接触物体（需按压力传感器实际标定） */
#define FINGER_CONTACT_FORCE_THRESHOLD 30.0f


#define FINGER_MAX_FORCE 1000.0f  //最大抓取力（需按压力传感器实际标定）

/* raw_to_force / voltage_to_force_v 标定参数（需按压力传感器实际标定） */
#define FINGER_FORCE_RAW_FULL        4095.0f
#define FINGER_FORCE_SEG1_RAW        320     /* raw 大于该值用第一段线性 */
#define FINGER_FORCE_SEG1_SLOPE      14.0f
#define FINGER_FORCE_SEG2_RAW        68      /* raw 大于该值用第二段幂函数 */
#define FINGER_FORCE_SEG2_SLOPE      16.4f
#define FINGER_FORCE_SEG2_EXP        1.15f
#define FINGER_FORCE_SEG3_SLOPE      2.19f
#define FINGER_FORCE_SEG3_EXP        1.75f
#define FINGER_FORCE_MAX             10000.0f

/* voltage_to_force_v 备用标定参数 */
#define FINGER_FORCE_V_OFFSET        0.008f
#define FINGER_FORCE_V_SLOPE         0.0008f

/* catch 前是否必须检测到接触（contacted）。
 * 1 = 严格防空抓；0 = 允许空抓（调试/闭手），
 * 安全由 CATCHING 超时 + 抗积分饱和 + FINGER_MAX_FORCE 兜底。 */
#ifndef FINGER_CATCH_REQUIRE_CONTACT
#define FINGER_CATCH_REQUIRE_CONTACT  0
#endif

/* 抓取方向：两根手指镜像安装时，手指1 增大角度闭合、手指2 减小角度闭合。
 * 若你的机械是两根手指同向开合，把 FINGER2_GRASP_DIR 改为 +1。 */
#define FINGER1_GRASP_DIR  (+1)
#define FINGER2_GRASP_DIR  (-1)
#define FINGER3_GRASP_DIR  (+1)

/* 位置指令（approach/move）的角度按“闭合量”解释：0=全开，180=全闭。
 * 镜像安装时 grasp_dir<0 的手指自动换算为 180-angle；同向安装设为 0。 */
#ifndef FINGER_POSITION_MIRROR
#define FINGER_POSITION_MIRROR  1
#endif

/* 每根手指机械允许的角度范围（按实际结构标定，默认 0-180） */
#define FINGER1_MIN_DEG   0.0f
#define FINGER1_MAX_DEG   180.0f
#define FINGER2_MIN_DEG   0.0f
#define FINGER2_MAX_DEG   180.0f
#define FINGER3_MIN_DEG   0.0f
#define FINGER3_MAX_DEG   180.0f

/* 抓取 PID 参数（需按机械/压力传感器标定） */
#define FINGER_PID_KP               0.15f
#define FINGER_PID_KI               0.002f
#define FINGER_PID_KD               0.0f
#define FINGER_PID_MAX_OUTPUT       2.0f    /* 每标称周期(5ms)角度增量限幅，内部按实际 dt 缩放 */
#define FINGER_PID_MAX_INTEGRAL     300.0f
#define FINGER_PID_INTEGRAL_START   200.0f  /* 误差小于该值才开始积分 */

/* CATCHED 判定：误差在容差内连续 N 个节拍则认为抓取到位 */
#define FINGER_CATCHED_FORCE_TOL     10.0f
#define FINGER_CATCHED_STABLE_TICKS  10

/* CATCHED 保力：误差超过 2 倍容差并持续 N 拍则认为物体滑落，回到 CATCHING 重抓 */
#define FINGER_CATCHED_LOST_FORCE_TOL  (FINGER_CATCHED_FORCE_TOL * 2.0f)
#define FINGER_CATCHED_LOST_TICKS      20

/* CATCHING 超时：超过该时间仍未到位则放弃并保持位置，避免一直堵转 */
#define FINGER_CATCH_TIMEOUT_MS      3000
#define FINGER_CATCH_TIMEOUT_TICKS   (FINGER_CATCH_TIMEOUT_MS / FINGER_UPDATE_PERIOD_MS)

/* 舵机接 PCA9685 的 PWM 通道号（0-15） */
#define FINGER1_CHANNEL  0
#define FINGER2_CHANNEL  1
#define FINGER3_CHANNEL  2

/* 角度变化小于该值（度）时不重复写 PCA9685（约小于 1 个计数 0.44°） */
#ifndef SERVO_WRITE_DEADBAND_DEG
#define SERVO_WRITE_DEADBAND_DEG  0.2f
#endif


#endif
