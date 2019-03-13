#ifndef ROBOTCONFIG_H_
#define ROBOTCONFIG_H_
//#include <aris.h>
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
		std::array<double, 6> forceTransform(std::array<double, 6> &q, std::array<double, 3> &f, std::array<double, 3> &m);
	};

	    const double ConRad = PII / 180;
        const double ConAng = 180 / PII;
        const int GroupDim = 46;
        const int RobotAxis = 6;
        const double DT = 0.008;
        const double CutoffFrequency = 5;
        const double EndPoint[3] = { 0, 0, 0.078};
        const double EndSensor[3] = { 0, 0, 0};
        
        const double ZeroOffset[6] = { 0, 0, 0, 0, 0, 0 };
        const double JointOffset[6] = { 0, -PII / 2, 0, 0, 0, 0 };
        const double DirectionFlag[6] = { 1, 1, 1, 1, 1, 1 };

		//DH Paras
		const double a2 = 0.04,a3=0.275,a4=0.025,d4=0.28;
}

#endif
