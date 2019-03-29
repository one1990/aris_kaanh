#ifndef SIXDISTALIDENTIFY_H_
#define IIR_H_

#include <vector>

# define FORE_VEL_LENGTH 20	//速度平均值滤波buffer长度
# define PI 3.1415926	//中值滤波buffer长度

namespace sixDistalIdentifyInt
{ 
	class sixdistalidentify
	{

        double estParas[6];
       
        double cutoffFreq = 2 * 3.14 * CutoffFrequency;
        double A[3][3];
        double B[3];
        double q[6];
        double dq[6];
        double ddq[6];
        double ts[6];
        //List<double[,]> matrixList;
        //List<double[]> forceList;
       // Matrix regressorMatrix;
        //Matrix regressorForces;

        //sixDistalDynamics sixDistalMatrix;

        sixdistalidentify();
		std::array<double, 6> RLS(std::array<double, 6> &positionList, std::array<double, 6> &sensorList);
		
	};
	/*
        const double ConRad = PI / 180;
                const double ConAng = 180 / PI;
                const int GroupDim = 13;
                const int RobotAxis = 6;
                const double DT = 0.008;
                const double CutoffFrequency = 5;
                const double EndPoint[3] = { 0, 0, 0.106 };
                const double EndSensor[3] = { 0, 0, 0};

                const double ZeroOffset[6] = { 0, 0, 0, 0, 0, 0 };
                const double JointOffset[6] = { 0, -PI / 2, 0, 0, 0, 0 };
                const double DirectionFlag[6] = { 1, -1, -1, 1, 1, 1 };

		//DH Paras
		const double a2 = 0.008,a3=0,a4=0,d4=0,DF = EndPoint[2];;*/
}

#endif
