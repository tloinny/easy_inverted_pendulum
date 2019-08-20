#include <easy_inverted_pendulum.h>

inverted_pendulum::inverted_pendulum(float A_Kp, float A_Kd, float P_Kp, float P_Kd, long A_st, long P_st, int A_limit, int P_limit)
{
	Angle_loop(A_Kp, A_Kd);
	Position_loop(P_Kp, P_Kd);
	SetAngleSetPoint(0);
	SetPositionSetPoint(0);

	this->AngleDTerm = 0;
	this->AngleSetPoint = 0;
	this->AngleError = 0;
	this->AngleLastError = 0;
	this->AngleOutput = 0;

	this->PositionDTerm = 0;
	this->PositionSetPoint = 0;
	this->PositionError = 0;
	this->PositionLeast = 0;
	this->PositionLastError = 0;
	this->PositionOutput = 0;	

	this->AngleOutputLimit = A_limit;
	this->PositionOutputLimit = P_limit;

	this->AngleLoopSampleTime = A_st;
	this->PositionLoopSampleTime = P_st;

	AngleCurrentTime = millis();
	PositionCurrentTime = AngleCurrentTime;
	AngleLastTime = AngleCurrentTime;
	PositionLastTime = AngleCurrentTime;
}

void inverted_pendulum::Angle_loop(float Angle_kp, float Angle_kd)
{
	this->angle_kp = Angle_kp;
	this->angle_kd = Angle_kd;
}

void inverted_pendulum::Position_loop(float Position_kp, float Position_kd)
{
	this->position_kp = Position_kp;
	this->position_kd = Position_kd;
}

float inverted_pendulum::AngleLoopUpdate(float angle_feedback)
{
	AngleError = angle_feedback - AngleSetPoint;	/* calculate the angle error */
	AngleCurrentTime = millis();
	long delta_time = AngleCurrentTime - AngleLastTime;
	if (delta_time >= AngleLoopSampleTime && delta_time > 0)
	{
		AngleLastTime = AngleCurrentTime;
		AngleDTerm = (AngleError - AngleLastError) / delta_time;	/* calculate the differential of angle error */
		AngleOutput = -1 * (angle_kp * AngleError + angle_kd * AngleDTerm);	/* calculate output */
		AngleLastError = AngleError;	/* update AngleLastError */
	}
	return AngleOutput;
}

float inverted_pendulum::PositionLoopUpdate(float position_feedback)
{
	PositionLeast = position_feedback - PositionSetPoint;
	PositionError *= 0.8;
	PositionError += PositionLeast * 0.2;	/* low pass filter */
	PositionCurrentTime = millis();
	long delta_time = PositionCurrentTime - PositionLastTime;
	if (delta_time >= PositionLoopSampleTime && delta_time > 0)
	{
		PositionLastTime = PositionCurrentTime;
		PositionDTerm = (PositionError - PositionLastError) / delta_time;
		PositionOutput = position_kp * PositionError + position_kd * PositionDTerm;
		PositionLastError = PositionError;
	}
	return PositionOutput;
}

float inverted_pendulum::InvertedPendulumUpdate(float angle_encoder, float position_encoder)
{
	AngleLoopUpdate(angle_encoder);
	PositionLoopUpdate(position_encoder);
	/* range limit */

	if (AngleOutput > 0)
	{
		AngleOutput += 255 * 0.05;
	}else if (AngleOutput < 0)
	{
		AngleOutput -= 255 * 0.05;
	}

	if (AngleOutput > AngleOutputLimit)
	{
		AngleOutput = AngleOutputLimit;
	}else if (AngleOutput < -1 * AngleOutputLimit)
	{
		AngleOutput = -1 * AngleOutputLimit;
	}

	if (PositionOutput > PositionOutputLimit)
	{
		PositionOutput = PositionOutputLimit;
	}else if (PositionOutput < -1 * PositionOutputLimit)
	{
		PositionOutput = -1 * PositionOutputLimit;
	}

	InvertedPendulumOutput = AngleOutput + PositionOutput;

	if (InvertedPendulumOutput > 255)
	{
		InvertedPendulumOutput = 255;
	}else if (InvertedPendulumOutput < -255)
	{
		InvertedPendulumOutput = -255;
	}

	return InvertedPendulumOutput;
}

void inverted_pendulum::SetAngleSetPoint(float setpoint)
{
	this->AngleSetPoint = setpoint;
}

void inverted_pendulum::SetPositionSetPoint(float setpoint)
{
	this->PositionSetPoint = setpoint;
}

float inverted_pendulum::GetAngleSetPoint()
{
	return AngleSetPoint;
}

float inverted_pendulum::GetPositionSetPoint()
{
	return PositionSetPoint;
}
