#ifndef SIXDISTALDYNAMICS_H_
#define SIXDISTALDYNAMICS_H_

#include <array>


namespace sixDistalDynamicsInt
{ 
	class sixdistaldynamics
	{
	public:
		double A[3][3];
		double B[3];
        sixdistaldynamics();
		std::array<double, 6> sixDistalCollision(std::array<double, 6> &q, std::array<double, 6> &dq, std::array<double, 6> &ddq, std::array<double, 6> &ts, std::array<double, 6> &estParas);
		void RLS(const double *positionList, const double *sensorList, double *estParas);
		//double[,] sixDistalMatrix(double[] q, double[] dq,double[] ddq,double[] ts);
		
	};
}

#endif
