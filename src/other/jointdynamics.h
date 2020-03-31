#ifndef JOINTDYNAMICS_H_
#define JOINTDYNAMICS_H_

#include <array>


namespace JointDynamicsInt
{ 
    const int LoadTotalParas = 40;
    const int LoadReduceParas = 13;
    const int JointGroupDim = 60;
    const int JointReduceDim = 30;
   

	class jointdynamics
	{
	public:

		double A[3][3];
		double B[3];
	
	double estParasLYang[LoadTotalParas + 6] = { 0 };
	double estParasL0Yang[LoadTotalParas + 6] = { 0 };

    double estParasL[LoadReduceParas+6] = { 0 };
	double CoefParasL[LoadReduceParas* LoadTotalParas] = { 0 };
	double estParasL0[LoadReduceParas+6] = { 0 };
	double CoefParasL0[LoadReduceParas* LoadTotalParas] = { 0 };
    double LoadParas[10] = { 0 };
    double Load2Joint[13*10] = { 0 };
	double CoefParasLoad[13* 40] = { 0 };
	double CoefParasLoadInv[13* 40] = { 0 };

	double estParasJoint0[JointReduceDim+12] = { 0 };
	double estParasJoint[JointReduceDim + 12] = { 0 };
	double estParasJointYang[JointGroupDim + 12] = { 0 };
	double CoefParasJoint[JointReduceDim* JointGroupDim] = { 0 };
	double CoefParasJointInv[JointReduceDim* JointGroupDim] = { 0 };

	
        jointdynamics();
        void JointCollision(const double * q, const double *dq,const double *ddq,const double *ts, const double *estParas, const double * CoefInv, const double * Coef, const double * LoadParas, double * CollisionFT, const double* Acv);
		void JointCollisionYang(const double * q, const double *dq, const double *ddq, const double *ts, const double *estParas, const double * LoadParas, double * CollisionFT, const double* Acv);
		void JointCollisionAris(const double * q, const double *dq,const double *ddq,const double *ts, const double *estParas, const double * CoefInv, const double * Coef, const double * LoadParas, double * CollisionFT, const double* Acv);

        void JointDrag(const double * q, const double *dq, const double *ddq, const double *ts, const double *estParas, const double * CoefInv, const double * Coef, const double * LoadParas, double * CollisionFT, const double* Acv);
		void JointDragYang(const double * q, const double *dq, const double *ddq, const double *ts, const double *estParas, const double * LoadParas, double * CollisionFT, const double* Acv);

		void RLS(const double *positionList, const double *sensorList, double *estParas, double *Coef, double *CoefInv, double *StatisError);
		void RLStemp(const double *positionList, const double *sensorList, double *estParas, const double *Coef, const double *CoefInv, double *StatisError);
		void RLSaris(const double *positionList, const double *sensorList, double *estParas, double *Coef, double *CoefInv, double *StatisError);
        void RLSYang(const double *positionList, const double *sensorList, double *estParas, double *StatisError);


		void LoadRLS(const double *positionList, const double *sensorList, const double *Coef, const double *CoefInv,double *estParas, double *StatisError);
		void LoadRLStemp(const double *positionList, const double *sensorList, const double *Coef, const double *CoefInv, double *estParas, double *StatisError);
		void LoadRLSYang(const double *positionList, const double *sensorList, double *estParas, double *StatisError);


		void YYbase(const double *AngList, const double *VelList, const double *AccList, double *Load2Joint, double *Coef, double*CoefInv,const int TestNum);
		void LoadParasExt(const double *dEst, const double *Coef, const double*CoefInv, double *Load);
		void LoadParasExtYang(const double *positionL, const double *sensorL, const double * estParasL0, double* Load, double *StatisError);

		//void sixDistalMatrix(const double * q, const double *dq,const double *ddq,const double *ts,double );
		
	};

	
}

#endif
