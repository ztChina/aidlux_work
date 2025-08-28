typedef struct {
    float v_left;     // 左轮线速度 (m/s)
    float v_right;    // 右轮线速度 (m/s)
    float temp;       // 温度
    float humidity;   // 湿度
    float water_level;// 水位
} RobotStatus_t;
RobotStatus_t robot_status;