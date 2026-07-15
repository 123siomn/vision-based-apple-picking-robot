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
	pidVoltage.Kp = -0.15f;
	pidVoltage.Ki = 0.0f;
	pidVoltage.Kd = 0.0f;

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

/**
* @brief  限幅辅助函数
* @note   用于限制 PID 积分项和最终输出，避免电机异常时持续加速。
* @param  value: 待限制的数值
* @param  limit: 正负限幅值
* @return 限幅后的数值
*/
static float PID_LimitValue(float value, float limit)
{
	if(limit <= 0.0f)
	{
		return value;
	}
	if(value > limit)
	{
		return limit;
	}
	if(value < -limit)
	{
		return -limit;
	}
	return value;
}

/**
* @brief  清零 PID 运行状态
* @note   停车或切换模式时调用，防止旧积分继续影响下一次启动。
* @param  pid: PID 结构体指针
* @return 无
*/
void PID_Reset(tPid *pid)
{
	if(pid == 0)
	{
		return;
	}
	pid->actual_val = 0.0f;
	pid->err = 0.0f;
	pid->err_last = 0.0f;
	pid->err_sum = 0.0f;
}

/**
* @brief  带积分限幅和输出限幅的 PID 控制函数
* @note   主要用于电机速度闭环，避免编码器异常或反馈方向错误时输出持续增大。
* @param  pid: PID 结构体指针
* @param  actual_val: 当前反馈值
* @param  output_limit: 最终输出正负限幅
* @param  err_sum_limit: 积分累计正负限幅
* @return 限幅后的 PID 输出
*/
float PID_realize_limit(tPid *pid, float actual_val, float output_limit, float err_sum_limit)
{
	float output;

	pid->actual_val = actual_val;
	pid->err = pid->target_val - pid->actual_val;
	pid->err_sum += pid->err;
	pid->err_sum = PID_LimitValue(pid->err_sum, err_sum_limit);

	output = pid->Kp * pid->err + pid->Ki * pid->err_sum + pid->Kd * (pid->err - pid->err_last);
	pid->err_last = pid->err;
	pid->actual_val = PID_LimitValue(output, output_limit);
	return pid->actual_val;
}
