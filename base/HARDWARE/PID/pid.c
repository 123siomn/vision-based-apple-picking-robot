#include "pid.h"

// 底盘当前保留的 PID 实例：电机速度、灰度循迹和 MPU6050 航向控制。
tPid pidMotor1Speed;          // 电机1速度PID闭环参数
tPid pidMotor2Speed;          // 电机2速度PID闭环参数
tPid pidVoltage;              // 四路灰度循迹PID参数
tPid pidMPU6050YawMovement;   // MPU6050航向角PID参数

// 给结构体类型变量赋初值
void PID_init(void)
{
	pidMotor1Speed.actual_val = 0.0f;
	pidMotor1Speed.target_val = 0.0f;
	pidMotor1Speed.err = 0.0f;
	pidMotor1Speed.err_last = 0.0f;
	pidMotor1Speed.err_sum = 0.0f;
	pidMotor1Speed.Kp = 15.0f;
	pidMotor1Speed.Ki = 5.0f;
	pidMotor1Speed.Kd = 0.0f;

	pidMotor2Speed.actual_val = 0.0f;
	pidMotor2Speed.target_val = 0.0f;
	pidMotor2Speed.err = 0.0f;
	pidMotor2Speed.err_last = 0.0f;
	pidMotor2Speed.err_sum = 0.0f;
	pidMotor2Speed.Kp = 15.0f;
	pidMotor2Speed.Ki = 5.0f;
	pidMotor2Speed.Kd = 0.0f;

	pidVoltage.actual_val = 0.0f;
	pidVoltage.target_val = 0.0f; // 灰度循迹PID的目标误差为0
	pidVoltage.err = 0.0f;
	pidVoltage.err_last = 0.0f;
	pidVoltage.err_sum = 0.0f;
	pidVoltage.Kp = -1.00f;
	pidVoltage.Ki = 0.0f;
	pidVoltage.Kd = -0.2f;

	pidMPU6050YawMovement.actual_val = 0.0f;
	pidMPU6050YawMovement.target_val = 0.0f; // 航向角PID目标值
	pidMPU6050YawMovement.err = 0.0f;
	pidMPU6050YawMovement.err_last = 0.0f;
	pidMPU6050YawMovement.err_sum = 0.0f;
	pidMPU6050YawMovement.Kp = 0.02f;
	pidMPU6050YawMovement.Ki = 0.0f;
	pidMPU6050YawMovement.Kd = 0.1f;
}

// 比例P调节控制函数
float P_realize(tPid *pid, float actual_val)
{
	pid->actual_val = actual_val;
	pid->err = pid->target_val - pid->actual_val;
	pid->actual_val = pid->Kp * pid->err;
	return pid->actual_val;
}

// 比例P + 积分I控制函数
float PI_realize(tPid *pid, float actual_val)
{
	pid->actual_val = actual_val;
	pid->err = pid->target_val - pid->actual_val;
	pid->err_sum += pid->err;
	pid->actual_val = pid->Kp * pid->err + pid->Ki * pid->err_sum;
	return pid->actual_val;
}

// PID控制函数
float PID_realize(tPid *pid, float actual_val)
{
	pid->actual_val = actual_val;
	pid->err = pid->target_val - pid->actual_val;
	pid->err_sum += pid->err;
	pid->actual_val = pid->Kp * pid->err + pid->Ki * pid->err_sum + pid->Kd * (pid->err - pid->err_last);
	pid->err_last = pid->err;
	return pid->actual_val;
}
