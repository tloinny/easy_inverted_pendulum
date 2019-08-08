#ifndef EASY_INVERTED_PENDULUM
#define EASY_INVERTED_PENDULUM

#include <Arduino.h>

class inverted_pendulum
{
public:
	inverted_pendulum(float A_Kp, float A_Kd, float P_Kp, float P_Kd, long A_st, long P_st, int A_limit, int P_limit);
	void Angle_loop(float Angle_kp, float Angle_kd);
	void Position_loop(float Position_kp, float Position_kd);
	void SetAngleSetPoint(float setpoint);
	void SetPositionSetPoint(float setpoint);
	float AngleLoopUpdate(float angle_feedback);
	float PositionLoopUpdate(float position_feedback);
	float InvertedPendulumUpdate(float angle_encoder, float position_encoder);
	float GetAngleSetPoint();
	float GetPositionSetPoint();
private:
	float angle_kp;
	float angle_kd;
	float AngleDTerm;
	float AngleSetPoint;
	float AngleError;
	float AngleLastError;
	float AngleOutput;
	long AngleLoopSampleTime;
	long AngleCurrentTime;
	long AngleLastTime;

	float position_kp;
	float position_kd;
	float PositionDTerm;
	float PositionSetPoint;
	float PositionError;
	float PositionLeast;
	float PositionLastError;
	float PositionOutput;
	long PositionLoopSampleTime;
	long PositionCurrentTime;
	long PositionLastTime;

	float InvertedPendulumOutput;
	float AngleOutputLimit;
	float PositionOutputLimit;
};

#endif