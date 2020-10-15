# small_4_legged
小型四足机器人
控制板: Arduino nano
舵机: SG90
舵机驱动板：16路PWM
imu: mpu-6050

接线：
nano        舵机板
AIN4 ------- SDA
AIN5 ------- SCL
5V -------- VCC
GND -------- GND

电源        舵机板
5V -------- VCC
5V -------- V+
GND -------- GND

nano无法同时驱动8个舵机，舵机板的V+不能由nano板提供，可能是nano板无法提供足够的功率
8个舵机同时使用的电流大概在1A