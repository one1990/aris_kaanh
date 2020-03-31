#ifndef ROBOTCONFIG_H_
#define ROBOTCONFIG_H_
#include <aris.hpp>
#include <array>

# define FORE_VEL_LENGTH 20	//�ٶ�ƽ��ֵ�˲�buffer����
# define PII 3.1415926	//��ֵ�˲�buffer����

namespace CONFIG
{ 
	class robotconfig
	{
                public:
                double A0[6][6],B0[4][4];
                double dTheta[6];
                double MC1, MC2, MC3;

		robotconfig();
		void jointIncrement(const double* q, const double* dX, double* dTheta);
		void forceTransform(const double* q, const float* FmInEnd, double* FmInWorld);

	};

        const int SampleNum = 6000;
	    const double ConRad = PII / 180;
        const double ConAng = 180 / PII;

		const double DT = 0.001;
        const double CutoffFrequency = 5;
		const double EndPoint[3] = { 0, 0, 0.078};//The Position of End Plate
        const double EndSensor[3] = { 0, 0, 0.1155};//The Position of Torque Sensor

		//aris::plan::PlanTarget

		//六轴工业
        const int RobotAxis = 6;
        const double ZeroOffset[6] = { 0, 0, 0, 0, 0, 0 };
        const double JointOffset[6] = { 0, -PII / 2, 0, 0, 0, 0 };
        const double DirectionFlag[6] = { 1, 1, 1, 1, 1, 1 };
        const double a2 = 0.04,a3=0.275,a4=0.025,d3=0,d4=0.28,d5 = 0;//Rokae DH
        //const double a2 = 0.088,a3=0.46,a4=0.04,d3=0,d4=0.43,d5 = 0;//MJ08 DH
        /*
		//七轴
		const int RobotAxis = 7;
		const double ZeroOffset[7] = { 0, 0, 0, 0, 0, 0 ,0};
		const double JointOffset[7] = { 0, -PII, -PII, -PII, -PII, -PII ,-PII};
        const double DirectionFlag[7] = { 1, -1, 1, -1, 1, -1 ,1};
		const double a2=0, a3=0, a4=0, d4=0, d3 = 0.33, d5 = 0.32, d7 = 0.28;
        */

		
}

#endif
