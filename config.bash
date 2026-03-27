#!/bin/bash
export GLOBAL_ACC_VEL_THETA=0.4       #角加速度
export GLOBAL_ACC_VEL_X=0.4           #线加速度
export GLOBAL_MAX_VEL_THETA=0.4       #最大角速度
export GLOBAL_MAX_VEL_X=0.4           #最大线速度
export GLOBAL_DEFAULT_MAX_VEL_X=0.3   #默认线速度
export GLOBAL_LASER_AMOUNT=2          #激光雷达数量    
# 激光雷达1位姿参数
export GLOBAL_LASER_1_POSE_X=0.260
export GLOBAL_LASER_1_POSE_Y=0.0
export GLOBAL_LASER_1_POSE_Z=0.3
export GLOBAL_LASER_1_POSE_ROLL=0
export GLOBAL_LASER_1_POSE_PITCH=0.0   
export GLOBAL_LASER_1_POSE_YAW=3.1415926
#  激光雷达2位姿参数
export GLOBAL_LASER_2_POSE_X=-0.260
export GLOBAL_LASER_2_POSE_Y=0.0
export GLOBAL_LASER_2_POSE_Z=0.3
export GLOBAL_LASER_2_POSE_ROLL=0
export GLOBAL_LASER_2_POSE_PITCH=0.0   
export GLOBAL_LASER_2_POSE_YAW=0
# imu位姿参数
# H13或H02
export GLOBAL_IMU_TYPE=H13
export GLOBAL_IMU_X=0.213
export GLOBAL_IMU_Y=0.0
export GLOBAL_IMU_Z=0.15
export GLOBAL_IMU_YAW=0.0
export GLOBAL_IMU_PITCH=0.0
export GLOBAL_IMU_ROLL=0.0

# 超声波传感器位姿参数（多个，以3个为例)
export GLOBAL_SONAR_AMOUNT=3

export GLOBAL_SONAR_0_X=0.246
export GLOBAL_SONAR_0_Y=0.172
export GLOBAL_SONAR_0_Z=0.15
export GLOBAL_SONAR_0_YAW=0.611
export GLOBAL_SONAR_0_PITCH=0.0
export GLOBAL_SONAR_0_ROLL=0.0

export GLOBAL_SONAR_1_X=0.3
export GLOBAL_SONAR_1_Y=0.0
export GLOBAL_SONAR_1_Z=0.15
export GLOBAL_SONAR_1_YAW=0.0
export GLOBAL_SONAR_1_PITCH=0.0
export GLOBAL_SONAR_1_ROLL=0.0

export GLOBAL_SONAR_2_X=0.246
export GLOBAL_SONAR_2_Y=-0.172
export GLOBAL_SONAR_2_Z=0.15
export GLOBAL_SONAR_2_YAW=-0.611
export GLOBAL_SONAR_2_PITCH=0.0
export GLOBAL_SONAR_2_ROLL=0.0

# 机器人类型
export GLOBAL_ROBOT_NAME=stocktaking
# 机器人版本
export GLOBAL_ROBOT_VERSION=1.0

# 磁条和TOF
export MAGNETIC_ENABLE=1
export MAGNETIC_SHUTDOWN_ENABLE=0
export TOF_SHUTDOWN_ENABLE=0

#升降杆卡死关机
export LIFT_JAMMED_SHUTDOWN_ENABLE=0

#是否发送异常事件邮件
export SENDMAIL_ENABLE=1
#发送给多个接收方(最多8个)
export RECEIVE_MAIL_AMOUNT=2
export RECEIVE_MAIL1=wangzx@sis-bot.com
export RECEIVE_MAIL2=jiym@sis-bot.com

# 迷路关机功能
# 是否开启迷路关机功能
export LOST_SHUTDOWN_ENABLE=1
# 迷路关机功能的参数
# 机器人迷路多少秒后关机
export LOST_SHUTDOWN_TIME=2.0
# 机器人迷路多少米后关机
export LOST_SHUTDOWN_DISTANCE=0.5
# 机器人迷路角度转多少度后关机
export LOST_SHUTDOWN_ANGLE=90.0
# 机器人迷路时的定位质量阈值，低于此值认为迷路
export LOST_SHUTDOWN_LOCALIZATION_QUALITY=40

# 机器人自动充电功能参数
# 充上电时，机器人边缘到充电桩中心的距离
# 如果回充过早停止，可以适当减小此值
# 如果回充，充上了还继续后退，可以适当增大该值
export AUTO_CHANGE_CENTER_TO_ROBOT_DISTANCE=0.16
# 当机器人和充电桩中心的距离达到AUTO_CHANGE_CENTER_TO_ROBOT_DISTANCE时，
# 机器人继续后退AUTO_CHANGE_REACHED_TIME_DELAY的时间，防止机器人顶上去的力不够大
# 如果机器人顶上去的力够大，可以适当减小此值
# 如果机器人顶上去的力不够大，可以适当增大此值
export AUTO_CHANGE_REACHED_TIME_DELAY=0.4

# 盘点直线货架时，前方障碍物停止距离
export LINE_SHELF_COLLISION_CHECK_DISTANCE=0.25

# 盘点异形货架时，因为书架终端有一些不需要跟随的点，需要移除终端一定距离的点，不跟随这些点
# 如果没有需要移除的点，可以设置为0
# 帕莎馆该参数设置为0.16 以为异形书架的终点时类似下图.BC段(大致16CM长)不需要跟随。
#                                      C
#  \                                  /
#   \--------------------------------/
#   A                               B
export CURVE_SHELF_REMOVE_LAST_POINTS_LENGTH=0.0
# 盘点异形货架时,在终点处停止的距离。
# 0表示在终点处停止即可
# 0.09表示在终点处再往前走0.09米后停止
# -0.09表示尚未到达终点，还有0.09米时即停止，认为到达终点
export CURVE_SHELF_STOP_GOAL_DIST=0.0


# frpc地址和端口的配置
# 国内:121.43.128.105
# 新加坡:47.236.196.89
export FRPC_IP=121.43.128.105
export FRPC_PORT=7000

#export USE_LINEAR=



# export FRONT_LIDAR_PORT="/dev/ttyUSB0"  # 实际前端雷达设备号
# export BACK_LIDAR_PORT="/dev/ttyUSB1"   # 实际后端雷达设备号