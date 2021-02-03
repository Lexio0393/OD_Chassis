User_Bsp存放底层配置程序
包含CAN、IO、Serial、Systick、Timer


User_Algorithm存放算法相关程序
包含
algorithm	（用户所需数学相关函数）
pid           （PID计算相关函数）
chassis	（四轮全向底盘的运动学解算）


User_App存放用户自定义应用
包含
C620        （电机控制函数）
pps	（编码器定位系统的配置即反馈信息）
bsp_09s	（WFLY09SⅡ遥控器）
task	（任务函数）


B为模式选择，F为遥控模式下模式选择


2020/12/08
待添加速度柔化
待添加定位器15s提示
待测试自动
运动学解算待验证
电机转向待调试


Test on main branch on Feb. 4
Test on main branch on Feb. 4 with username :Leixo_Lei