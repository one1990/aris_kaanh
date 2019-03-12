#ifndef SIXDISTALDYNAMICS_H_
#define SIXDISTALDYNAMICS_H_

#include <vector>

# define FORE_VEL_LENGTH 20	//速度平均值滤波buffer长度
# define PI 3.1415926	//中值滤波buffer长度

namespace sixDistalDynamicsInt
{ 
	class sixdistaldynamics
	{
	public:
		double A[3][3];
		double B[3];
        sixdistaldynamics();
		std::array<double, 6> sixDistalCollision(std::array<double, 6> &q, std::array<double, 6> &dq, std::array<double, 6> &ddq, std::array<double, 6> &ts, std::array<double, 6> &estParas);
		std::array<double, 6> RLS(std::array<double, 6> &positionList, std::array<double, 6> &sensorList);
		//double[,] sixDistalMatrix(double[] q, double[] dq,double[] ddq,double[] ts);
		
	};
}

#endif
