#ifndef SIXDISTALDYNAMICS_H_
#define SIXDISTALDYNAMICS_H_

#include <array>


namespace sixDistalDynamicsInt
{ 
   const int GroupDim = 16;
	class sixdistaldynamics
	{
	public:
        double estParasFT[GroupDim] = { 0 };
		double A[3][3];
		double B[3];
        sixdistaldynamics();
        void sixDistalCollision(const double * q, const double *dq,const double *ddq,const double *ts, const double *estParas, double * estFT);
        void RLS(const double *positionList, const double *sensorList, double *estParas, double *StatisError);
        //void sixDistalMatrix(const double * q, const double *dq,const double *ddq,const double *ts,double );
		
	};
}

#endif
