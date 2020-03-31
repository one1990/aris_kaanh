#ifndef SEVENJOINTDYNAMICS_H_
#define SEVENJOINTDYNAMICS_H_

#include <array>





namespace SevenJointDynamicsInt
{
	const int LoadTotalParas = 40;
	const int LoadReduceParas = 15;
	const int JointGroupDim = 70;
	const int JointReduceDim = 30;


	class sevenjointdynamics
	{
	public:

		double A[3][3];
		double B[3];

		double estParasL[LoadReduceParas + 6] = { 0 };
		double CoefParasL[LoadReduceParas* LoadTotalParas] = { 0 };
		double estParasL0[LoadReduceParas + 6] = { 0 };
		double CoefParasL0[LoadReduceParas* LoadTotalParas] = { 0 };
		double LoadParas[10] = { 0 };

		double estParasJoint0[JointReduceDim + 14] = { 0 };
		double estParasJoint[JointReduceDim + 14] = { 0 };
		double CoefParasJoint[JointReduceDim* JointGroupDim] = { 0 };
		double CoefParasJointInv[JointReduceDim* JointGroupDim] = { 0 };

		sevenjointdynamics();
		void SevenJointCollision(const double * q, const double *dq, const double *ddq, const double *ts, const double *estParas, double * CoefInv, double * CollisionFT);
		void SevenRLS(const double *positionList, const double *sensorList, double *estParas, double *Coef, double *CoefInv, double *StatisError);
		void SevenLoadRLS(const double *positionList, const double *sensorList, double *estParas, double *Coef, double *StatisError);
		void SevenLoadParasExt(const double *dEst, const double *dCoef, double *Load);

		//void sixDistalMatrix(const double * q, const double *dq,const double *ddq,const double *ts,double );

	};


}

#endif
