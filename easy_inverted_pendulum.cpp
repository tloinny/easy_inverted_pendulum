#include <easy_inverted_pendulum.h>

inverted_pendulum::inverted_pendulum(float A_Kp, float A_Kd, float P_Kp, float P_Kd, int A_limit, int P_limit)
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
	AngleDTerm = AngleError - AngleLastError;	/* calculate the differential of angle error */
	AngleOutput = angle_kp * AngleError + angle_kd * AngleDTerm;	/* calculate output */
	AngleLastError = AngleError;	/* update AngleLastError */
	AngleOutput *= -1;
	return AngleOutput;
}

float inverted_pendulum::PositionLoopUpdate(float position_feedback)
{
	PositionLeast = position_feedback - PositionSetPoint;
	PositionError *= 0.8;
	PositionError += PositionLeast * 0.2;	/* low pass filter */
	PositionDTerm = PositionError - PositionLastError;
	PositionOutput = position_kp * PositionError + position_kd * PositionDTerm;
	return PositionOutput;
}

float inverted_pendulum::InvertedPendulumUpdate(float angle_encoder, float position_encoder)
{
	AngleLoopUpdate(angle_encoder);
	PositionLoopUpdate(position_encoder);
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
