#include "pi_speed.h"

/**
 * @brief  将浮点数限制在正负指定范围内。
 * @param  value: 待限幅数值
 * @param  limit: 正负限幅绝对值
 * @return 限幅后的数值
 */
static float PiSpeed_LimitFloat(float value, float limit)
{
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
 * @brief  将 PWM 整数限制在正负指定范围内。
 * @param  value: 待限幅 PWM
 * @param  limit: PWM 正负限幅绝对值
 * @return 限幅后的 PWM
 */
static int16_t PiSpeed_LimitPwm(int16_t value, int16_t limit)
{
	if(value > limit)
	{
		return limit;
	}
	if(value < -limit)
	{
		return (int16_t)(-limit);
	}
	return value;
}

/**
 * @brief  初始化一个单轮 PI 速度控制器。
 * @param  controller: PI 控制器对象
 * @param  kp: 比例系数
 * @param  ki: 积分系数
 * @param  integralLimit: 积分项绝对值上限
 * @param  outputLimit: PWM 输出绝对值上限
 * @param  minActivePwm: 非零输出的最小有效 PWM，用于跨过电机死区
 * @return None
 */
void PiSpeed_Init(PiSpeedController_t *controller, float kp, float ki,
	float integralLimit, int16_t outputLimit, int16_t minActivePwm)
{
	if(controller == NULL)
	{
		return;
	}

	controller->kp = kp;
	controller->ki = ki;
	controller->integralLimit = integralLimit;
	controller->outputLimit = outputLimit;
	controller->minActivePwm = minActivePwm;
	PiSpeed_Reset(controller);
}

/**
 * @brief  清空目标速度、积分项和输出值。
 * @param  controller: PI 控制器对象
 * @return None
 */
void PiSpeed_Reset(PiSpeedController_t *controller)
{
	if(controller == NULL)
	{
		return;
	}

	controller->targetSpeed = 0.0f;
	controller->integral = 0.0f;
	controller->outputPwm = 0;
}

/**
 * @brief  设置单轮目标速度。
 * @param  controller: PI 控制器对象
 * @param  targetSpeed: 目标速度，单位由上层编码器换算定义
 * @return None
 */
void PiSpeed_SetTarget(PiSpeedController_t *controller, float targetSpeed)
{
	if(controller == NULL)
	{
		return;
	}

	/* 目标方向翻转时清积分，避免上一方向的积分造成反向突冲。 */
	if((controller->targetSpeed > 0.0f && targetSpeed < 0.0f) ||
		(controller->targetSpeed < 0.0f && targetSpeed > 0.0f))
	{
		controller->integral = 0.0f;
	}
	controller->targetSpeed = targetSpeed;
}

/**
 * @brief  使用当前实测速度执行一次 PI 运算。
 * @param  controller: PI 控制器对象
 * @param  measuredSpeed: 当前实测速度
 * @param  periodS: 本次控制周期，单位秒
 * @return 计算得到的 PWM 输出
 */
int16_t PiSpeed_Update(PiSpeedController_t *controller, float measuredSpeed, float periodS)
{
	float error;
	float candidateIntegral;
	float candidateOutput;
	float output;
	int16_t pwm;

	if(controller == NULL)
	{
		return 0;
	}
	if((controller->targetSpeed == 0.0f) || (periodS <= 0.0f))
	{
		controller->integral = 0.0f;
		controller->outputPwm = 0;
		return 0;
	}

	error = controller->targetSpeed - measuredSpeed;
	candidateIntegral = PiSpeed_LimitFloat(
		controller->integral + error * periodS, controller->integralLimit);
	candidateOutput = controller->kp * error + controller->ki * candidateIntegral;

	/* 输出已饱和且误差继续推向同一方向时，不再累积积分。 */
	if(!(((candidateOutput >= (float)controller->outputLimit) && (error > 0.0f)) ||
		((candidateOutput <= -(float)controller->outputLimit) && (error < 0.0f))))
	{
		controller->integral = candidateIntegral;
	}

	output = controller->kp * error + controller->ki * controller->integral;
	if(output >= 0.0f)
	{
		pwm = PiSpeed_LimitPwm((int16_t)(output + 0.5f), controller->outputLimit);
		if((pwm > 0) && (pwm < controller->minActivePwm))
		{
			pwm = controller->minActivePwm;
		}
	}
	else
	{
		pwm = PiSpeed_LimitPwm((int16_t)(output - 0.5f), controller->outputLimit);
		if((pwm < 0) && (pwm > -controller->minActivePwm))
		{
			pwm = (int16_t)(-controller->minActivePwm);
		}
	}

	controller->outputPwm = pwm;
	return pwm;
}

