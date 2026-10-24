控制律：
首先运行control-para.mlx获得轨迹数据并在此基础增加一段0.5s的常数改变角度初始值，获得qd，dqd，ddqd，并加载计算获得的鲁棒控制参数。
q.mat、dq.mat、ddq.mat、time.mat分别是通过轨迹规划计算获得的关节角度、、角速度、角加速度、时间。
tau.mat为关节力矩，由代码tau_generator利用逆动力学生成
controller.slx是SIMULINK搭建的控制器模型，鲁棒控制项、噪声项可以通过注释和取消注释添加和撤销。
