#ifndef __ATTITUDE_H
#define __ATTITUDE_H


typedef struct{
	float q0;
	float q1;
	float q2;
	float q3;
}quaterInfo_t;

typedef struct{
	float pitch;
	float roll;
	float yaw;
}eulerianAngles_t;
	
	


extern float values[10];

void IMU_getValues(float * values);
void IMU_quaterToEulerianAngles(void);

#endif
