#include "sixdistalfc.h"
#include <math.h>
#include"kaanh.h"
#include <algorithm>
#include <vector>
#include"jointdynamics.h"
#include"sixdistaldynamics.h"
//using namespace std;
using namespace aris::plan;
using namespace aris::dynamic;

JointDynamicsInt::jointdynamics JointMatrixFT;
sixDistalDynamicsInt::sixdistaldynamics sixDistalMatrix;

/// \brief
void crossVector(const double* a, const double* b, double* c)
{

    c[0] = a[1] * b[2] - b[1] * a[2];
    c[1] = -(a[0] * b[2] - b[0] * a[2]);
    c[2] = a[0] * b[1] - b[0] * a[1];

}

void GetATI(Plan *p,double* FT)
{
    int32_t FTint[6],status_code,sample_counter;

    #ifdef UNIX
    auto conSensor = dynamic_cast<aris::control::EthercatController*>(p->controller());

    conSensor->slavePool().at(6).readPdo(0x6000, 0x01, &FTint[0] ,32);
    conSensor->slavePool().at(6).readPdo(0x6000, 0x02, &FTint[1], 32);
    conSensor->slavePool().at(6).readPdo(0x6000, 0x03, &FTint[2], 32);
    conSensor->slavePool().at(6).readPdo(0x6000, 0x04, &FTint[3], 32);
    conSensor->slavePool().at(6).readPdo(0x6000, 0x05, &FTint[4], 32);
    conSensor->slavePool().at(6).readPdo(0x6000, 0x06, &FTint[5], 32);
    conSensor->slavePool().at(6).readPdo(0x6010, 0x00, &status_code, 32);
    conSensor->slavePool().at(6).readPdo(0x6020, 0x00, &sample_counter, 32);
    #endif

    double ATIscale=1000000.0;
    FT[0] = FTint[0]/ ATIscale;
    FT[1] = FTint[1] / ATIscale;
    FT[2] = FTint[2]/ ATIscale;
    FT[3] = FTint[3]/ ATIscale;
    FT[4] = FTint[4]/ ATIscale;
    FT[5] = FTint[5] / ATIscale;

}

void GetYuLi(Plan *p,double* FT)
{

    int16_t FTnum;
    float FTtemp[6];

    #ifdef UNIX
    auto conSensor = dynamic_cast<aris::control::EthercatController*>(p->controller());
    conSensor->slavePool().at(6).readPdo(0x6030, 0x00, &FTnum ,16);
    conSensor->slavePool().at(6).readPdo(0x6030, 0x01, &FTtemp[0] ,32);
    conSensor->slavePool().at(6).readPdo(0x6030, 0x02, &FTtemp[1], 32);
    conSensor->slavePool().at(6).readPdo(0x6030, 0x03, &FTtemp[2], 32);
    conSensor->slavePool().at(6).readPdo(0x6030, 0x04, &FTtemp[3], 32);
    conSensor->slavePool().at(6).readPdo(0x6030, 0x05, &FTtemp[4], 32);
    conSensor->slavePool().at(6).readPdo(0x6030, 0x06, &FTtemp[5], 32);
    #endif
   //std::cout<<FTtemp[0]<<"***"<<FTtemp[1]<<std::endl;
    for(int i=0;i<6;i++)
        FT[i]=FTtemp[i];

}

double Premax_pos[6]={0};
double Premin_pos[6]={0};
double Premax_vel[6]={0};
double Premin_vel[6]={0};
void SetLimit(Plan *p, double ratio)
{
    for(int i=0;i<6;i++)
    {
       Premax_pos[i]=p->controller()->motionPool()[i].maxPos();
       Premin_pos[i]= p->controller()->motionPool()[i].minPos();

       Premax_vel[i]= p->controller()->motionPool()[i].maxVel();
       Premin_vel[i]= p->controller()->motionPool()[i].minVel();

    }

        double max_pos[6]={2.96706/1, 1.57,1.0,  2.96706/3,1.57,6.28};
        double min_pos[6]={-2.96706/1,-0.5,     -1,  -2.96706/3,-1.57,-6.28};
        for(int i=0;i<6;i++)
        {
			p->controller()->motionPool()[i].setMaxVel(p->controller()->motionPool()[i].maxVel()/ratio);
			p->controller()->motionPool()[i].setMinVel(p->controller()->motionPool()[i].minVel()/ratio);
			p->controller()->motionPool()[i].setMaxPos(max_pos[i]);
			p->controller()->motionPool()[i].setMinPos(min_pos[i]);
        }

}

void ReSetLimit(Plan* p)
{
    for(int i=0;i<6;i++)
    {
		p->controller()->motionPool()[i].setMaxVel(Premax_vel[i]);
		p->controller()->motionPool()[i].setMinVel(Premin_vel[i]);
		p->controller()->motionPool()[i].setMaxPos(Premax_pos[i]);
		p->controller()->motionPool()[i].setMinPos(Premin_pos[i]);
    }

}

void FT2World(Plan *p,const double *FT, double *FmInWorld)
{
    double FT_YANG[6];
    FT_YANG[0] = -FT[0];FT_YANG[1] = -FT[1];FT_YANG[2] = FT[2];
    FT_YANG[3] = -FT[3];FT_YANG[4] = -FT[4];FT_YANG[5] = FT[5];

    double TransVector[16];
    p->model()->generalMotionPool().at(0).getMpm(TransVector);
    double TransMatrix[4][4];
    for (int i = 0;i < 4;i++)
        for (int j = 0;j < 4;j++)
            TransMatrix[i][j] = TransVector[4 * i + j];

    double n[3] = { TransMatrix[0][0], TransMatrix[0][1], TransMatrix[0][2] };
    double o[3] = { TransMatrix[1][0], TransMatrix[1][1], TransMatrix[1][2] };
    double a[3] = { TransMatrix[2][0], TransMatrix[2][1], TransMatrix[2][2] };

    //FT[0] = 0;FT[1] = 0;FT[2] = 1;FT[3] = 0;FT[4] = 0;FT[5] = 0;
    FmInWorld[0] = n[0] * FT_YANG[0] + n[1] * FT_YANG[1] + n[2] * FT_YANG[2];
    FmInWorld[1] = o[0] * FT_YANG[0] + o[1] * FT_YANG[1] + o[2] * FT_YANG[2];
    FmInWorld[2] = a[0] * FT_YANG[0] + a[1] * FT_YANG[1] + a[2] * FT_YANG[2];
    FmInWorld[3] = n[0] * FT_YANG[3] + n[1] * FT_YANG[4] + n[2] * FT_YANG[5];
    FmInWorld[4] = o[0] * FT_YANG[3] + o[1] * FT_YANG[4] + o[2] * FT_YANG[5];
    FmInWorld[5] = a[0] * FT_YANG[3] + a[1] * FT_YANG[4] + a[2] * FT_YANG[5];

}

void RepeatTrapezoidal(Plan *p,double *begin_pjs,double *step_pjs)
{
    static bool flag[6] = { true,true,true,true,true,true };
    double PosLimit[6] = { 0.000,0.40,0,0,0,0};
    double NegLimit[6] = { -0.000,-0.40,0,0,0,0};
    static double pArc[6], vArc[6], aArc[6], vArcMax[6] = { 0.001,0.4,0.001,0.001,0.001,0.001 };
    static aris::Size t_count[6] = { 0 };
    static int CountOffsetPos[6] = { 1,1,1,1,1,1 }, CountOffsetNeg[6] = { 1,1,1,1,1,1 };
    int temp[6] = { 0 };
    if(p->count()==1)
    {
        for(int i=0;i<6;i++)
        {
            flag[i]=true;
            t_count[i]=0;
            CountOffsetPos[i]=1;
            CountOffsetNeg[i]=1;
        }
    }
    for (int i = 0;i < 6;i++)
    {

        if (flag[i])
        {
            if (step_pjs[i] < PosLimit[i])
            {
                aris::plan::moveAbsolute(p->count() - CountOffsetNeg[i] + 1, 0, PosLimit[i] - begin_pjs[i], vArcMax[i] / 1000, 0.05 / 1000 / 1000, 0.05 / 1000 / 1000, pArc[i], vArc[i], aArc[i], t_count[i]);

                step_pjs[i] = step_pjs[i] + vArc[i];
            }
            //std::cout << vArc << "  ";
            if ((t_count[i] - (p->count() - CountOffsetNeg[i] + 1)) < 0.5 && (t_count[i] - (p->count() - CountOffsetNeg[i] + 1)) > -0.5)
            {
                CountOffsetPos[i] = p->count();
                flag[i] = false;
                begin_pjs[i] = step_pjs[i];//model()->motionPool()[i].mp();
            }


        }
        if (flag[i] == false)
        {
            if (step_pjs[i] > NegLimit[i])
            {
                aris::plan::moveAbsolute(p->count() - CountOffsetPos[i] + 1, 0, begin_pjs[i] - NegLimit[i], vArcMax[i] / 1000, 0.05 / 1000 / 1000, 0.05 / 1000 / 1000, pArc[i], vArc[i], aArc[i], t_count[i]);

                step_pjs[i] = step_pjs[i] - vArc[i];
            }

            temp[i] = t_count[i] - (p->count() - CountOffsetPos[i] + 1);
            if ((t_count[i] - (p->count() - CountOffsetPos[i] + 1)) < 0.5 && (t_count[i] - (p->count() - CountOffsetPos[i] + 1)) > -0.5)
            {
                CountOffsetNeg[i] = p->count();
                flag[i] = true;
                begin_pjs[i] = step_pjs[i];//model()->motionPool()[i].mp();
            }

        }
    }

}

void OneOrderFilter(const double *raw, const double *new0, double *new1, double CutFreq)
{

     for (int j = 0; j < 6; j++)
     {
         double intDT = 0.001;
         new1[j] = new0[j] + intDT * (raw[j]-new0[j])*CutFreq;
     }
}

void SecondOrderFilter(const double *raw, const double new0[][3], double new1[][3],double CutFreq)
{

    for (int j = 0; j < 6; j++)
    {
        double A[3][3], B[3];//SHANGHAI DIANQI EXP
        //CutFreq = 85;
        A[0][0] = 0; A[0][1] = 1; A[0][2] = 0;
        A[1][0] = 0; A[1][1] = 0; A[1][2] = 1;
        A[2][0] = -CutFreq * CutFreq * CutFreq;
        A[2][1] = -2 * CutFreq * CutFreq;
        A[2][2] = -2 * CutFreq;
        B[0] = 0; B[1] = 0;
        B[2] = -A[2][0];
        double intDT = 0.001;
        new1[j][0] = new0[j][0] + intDT * (A[0][0] * new0[j][0] + A[0][1] * new0[j][1] + A[0][2] * new0[j][2] + B[0] * raw[j]);
        new1[j][1] = new0[j][1] + intDT * (A[1][0] * new0[j][0] + A[1][1] * new0[j][1] + A[1][2] * new0[j][2] + B[1] * raw[j]);
        new1[j][2] = new0[j][2] + intDT * (A[2][0] * new0[j][0] + A[2][1] * new0[j][1] + A[2][2] * new0[j][2] + B[2] * raw[j]);
    }
}

void q2dqt(double *ActPq, const double *DesPq, double *dqt, const double KPP)
{
    double dQuar[4] = { 0 };
    //姿态误差2
    double cos_theta = ActPq[3] * DesPq[3] + ActPq[4] * DesPq[4] + ActPq[5] * DesPq[5] + ActPq[6] * DesPq[6];
    if (cos_theta < 0)
    {
        ActPq[3] = -ActPq[3];
        ActPq[4] = -ActPq[4];
        ActPq[5] = -ActPq[5];
        ActPq[6] = -ActPq[6];
    }
    cos_theta = ActPq[3] * DesPq[3] + ActPq[4] * DesPq[4] + ActPq[5] * DesPq[5] + ActPq[6] * DesPq[6];

    cos_theta = std::max(-1.0, cos_theta);
    cos_theta = std::min(1.0, cos_theta);

    double theta = std::acos(cos_theta);
    double sin_theta = std::sin(theta);


    if (theta < 0.03)
    {
        dQuar[0] = DesPq[3] - ActPq[3];
        dQuar[1] = DesPq[4] - ActPq[4];
        dQuar[2] = DesPq[5] - ActPq[5];
        dQuar[3] = DesPq[6] - ActPq[6];
    }
    else
    {
        dQuar[0] = -(DesPq[3] * cos_theta*(-theta) + theta * ActPq[3]) / sin_theta;
        dQuar[1] = -(DesPq[4] * cos_theta*(-theta) + theta * ActPq[4]) / sin_theta;
        dQuar[2] = -(DesPq[5] * cos_theta*(-theta) + theta * ActPq[5]) / sin_theta;
        dQuar[3] = -(DesPq[6] * cos_theta*(-theta) + theta * ActPq[6]) / sin_theta;
    }

    double norm_dQuar=std::max(1e-7, std::sqrt(aris::dynamic::s_vv(4,dQuar,dQuar)));
    double unit_dQuar[4]={0};

    for(int i=0;i<4;i++)
        unit_dQuar[i]=dQuar[i]/norm_dQuar;

    for(int i=0;i<4;i++)
    {
        double dt = std::min(KPP*theta, 0.5);// protect angular velocity target, theta always positive
        dqt[i]=unit_dQuar[i]*dt;
    }
}

void dX2dTheta(Plan *plan, const double *dXX, double *dTheta)
{
    double PqEnd[7];
    double EndW[3], EndP[3], BaseV[3];
	plan->model()->generalMotionPool().at(0).getMpq(PqEnd);

	double dX[6] = { 0 };
	for (int i = 0;i < 6;i++)
		dX[i] = dXX[i];

    ///* Using Jacobian, TransMatrix from ARIS
    for (int i = 0;i < 3;i++)
        EndW[i] = dX[i + 3];

    for (int i = 0;i < 3;i++)
        EndP[i] = PqEnd[i];
    crossVector(EndP, EndW, BaseV);

    for (int i = 0;i < 3;i++)
        dX[i + 3] = dX[i + 3];
    for (int i = 0;i < 3;i++)
        dX[i] = dX[i] + BaseV[i];


    auto &fwd = dynamic_cast<aris::dynamic::ForwardKinematicSolver&>(plan->model()->solverPool()[1]);
    fwd.cptJacobi();
    double pinv[36];

    // 所需的中间变量，请对U的对角线元素做处理
    double U[36], tau[6];
    aris::Size p[6];
    aris::Size rank;

    // 根据 A 求出中间变量，相当于做 QR 分解 //
    // 请对 U 的对角线元素做处理
    s_householder_utp(6, 6, fwd.Jf(), U, tau, p, rank, 1e-10);
    for (int i = 0;i < 6;i++)
        if (U[7 * i] >= 0)
            U[7 * i] = U[7 * i] + 0.001;
        else
            U[7 * i] = U[7 * i] - 0.001;
    // 根据QR分解的结果求x，相当于Matlab中的 x = A\b //
    s_householder_utp_sov(6, 6, 1, rank, U, tau, p, dX, dTheta, 1e-10);

    // 根据QR分解的结果求广义逆，相当于Matlab中的 pinv(A) //
    double tau2[6];
    s_householder_utp2pinv(6, 6, rank, U, tau, p, pinv, tau2, 1e-10);

    // 根据QR分解的结果求广义逆，相当于Matlab中的 pinv(A)*b //
    s_mm(6, 1, 6, pinv, dX, dTheta);
}

int signV(double x)
{
    double margin = 0.01;
	if (x > margin) return 1;
	if (abs(x) < margin || abs(x) == margin) return 0;
	if (x < -margin) return -1;
}

//系统传递函数H(s)=1/(ms)
void PIDcalOne(double m,double ts,double *KP)
{
	double T = ts / 3.0;
	KP[0] = m / T;
}

//系统传递函数H(s)=1/(ms+h)
void PIDcalTwo(double m, double h, double ts, double overshoot,double *KP, double *KI)
{
	double temp = log(overshoot);
	double kesi = 1 / sqrt(1 + aris::PI*aris::PI / temp / temp);
	double omega = 4 / kesi / ts;

	KI[0] = omega * omega * m;
	KP[0] = 2 * kesi *omega * m - h;
}

auto f(aris::dynamic::Model *m, double *A)->void
{
    auto &s = dynamic_cast<aris::dynamic::ForwardKinematicSolver&>(m->solverPool()[1]);

    // J_inv
    double U[36], tau[6], J_inv[36], tau2[6];
    aris::Size p[6], rank;	s_householder_utp(6, 6, s.Jf(), U, tau, p, rank, 1e-4);
    s_householder_utp2pinv(6, 6, rank, U, tau, p, J_inv, tau2, 1e-7);
    // M = (M + I) * J_inv
    double M[36], tem[36];
    s_mc(6, 6, s.M(), s.nM(), M, 6);
    for (int i = 0; i < 6; ++i)M[at(i, i, 6)] += m->motionPool()[i].frcCoe()[2];
    s_mm(6, 6, 6, M, J_inv, tem);
    s_mm(6, 6, 6, J_inv, T(6), tem, 6, A, 6);
}

static std::atomic_bool enable_FCPressL = true;
struct ForceDirectParam
{
	double PressF;
	double SensorType;
};
auto ForceDirect::prepareNrt()->void
{
	ForceDirectParam param;
    for (auto &p : cmdParams())
    {
        if (p.first == "PressF")
            param.PressF = doubleParam(p.first);
        if (p.first == "SensorType")
            param.SensorType = doubleParam(p.first);

    }
	this->param() = param;
    ret() = std::vector<std::pair<std::string, std::any>>();
    enable_FCPressL=true;

	for (auto &option : motorOptions()) option |=
		Plan::USE_TARGET_POS |
        Plan::NOT_CHECK_VEL_CONTINUOUS|
		Plan::NOT_CHECK_ENABLE;
}
auto ForceDirect::executeRT()->int
{
    auto &param = std::any_cast<ForceDirectParam&>(this->param());
    auto &lout = controller()->lout();

    static double begin_pjs[6]={0};
    static double step_pjs[6]={0};
    static double begin_t0[6];
    static double stateTor0[6], stateTor1[6];

    static double FT0[6];
    static double PqEnd0[7] = { 0 }, PqEnd[7] = { 0 },pe0[6] = { 0 },pe[6] = { 0 };
	// 访问主站 //
    auto &cout = controller()->mout();
    model()->generalMotionPool().at(0).getMpq(PqEnd);

	// 获取当前起始点位置 //
    double FT[6],FTemp[6];
    if(param.SensorType>0)
        GetATI(this,FT);
    else
        GetYuLi(this,FT);

    double q[6],dq[6],ddq[6],CollisionFT[6];
    for (int i = 0; i < 6; i++)
    {
        q[i]= controller()->motionPool()[i].actualPos();
        dq[i] =0;
        ddq[i] =0;
        FTemp[i]=FT[i];
    }

   // sixDistalMatrix.sixDistalCollision(q, dq, ddq, FT, sixDistalMatrix.estParasFT, CollisionFT);
    //for (int j = 0; j < 6; j++)
        //FT[j]=FT[j]-CollisionFT[j];


    if (count() == 1)
    {
        enable_FCPressL=true;
        SetLimit(this,4.0);
        for (int j = 0; j < 6; j++)
        {
            FT0[j]=FT[j];
        }
    }

    for (int j = 0; j < 6; j++)
    {
        FT[j]=FT[j]-FT0[j];
    }


    double FmInWorld[6]={0};
    FT2World(this,FT,FmInWorld);

    if (count() == 1)
    {
        for (int j = 0; j < 6; j++)
            stateTor0[j] = FmInWorld[j];
    }

    OneOrderFilter(FmInWorld,stateTor0,stateTor1,20);


	if (count() == 1)
	{
		for (int i = 0; i < 6; ++i)
		{
            controller()->motionPool().at(i).setModeOfOperation(10);	//切换到电流控制
		}
		for (int i = 0; i < 7; ++i)
		{
			PqEnd0[i] = PqEnd[i];
		}
		for (int i = 0; i < 6; ++i)
        {
            step_pjs[i] = 0;
            begin_t0[i] = 0;
            begin_pjs[i] = 0;
		}

        model()->generalMotionPool().at(0).getMpe(pe0);

	}	

    RepeatTrapezoidal(this,begin_pjs,step_pjs);

    aris::dynamic::s_vc(6, pe0, pe);
    pe[0]-=step_pjs[1]*0.08;
    //pe[3]+=step_pjs[1];
    //pe[4]+=step_pjs[1]*0.8;
    //pe[5]-=step_pjs[1]*0.2;
    aris::dynamic::s_pe2pq(pe, PqEnd0);


    //PqEnd0[1] = step_pjs[1];
    //PqEnd0[2] = begin_t0[2]+step_pjs[1]*0.2;
    //PqEnd0[2]=-15;

    ////////////////////////////////////////////////////compute dX(ee actual velocity)////////////////////////////////////////////////////////////////////////////////////////////////////////
    // dynamics //
    for (int i = 0; i < 6; ++i)
    {
        model()->motionPool()[i].setMp(controller()->motionPool()[i].actualPos());
        model()->motionPool()[i].setMv(controller()->motionAtAbs(i).actualVel());
        model()->motionPool()[i].setMa(0.0);
    }
	auto &fwd = dynamic_cast<aris::dynamic::ForwardKinematicSolver&>(model()->solverPool()[1]);
    if (fwd.kinPos())return -1;
    fwd.kinVel();
    fwd.cptJacobiWrtEE();
    fwd.cptGeneralInverseDynamicMatrix();
    model()->solverPool()[2].dynAccAndFce();

	double va[6] = { 0 }, dX[6] = { 0 };
	for (int i = 0;i < 6;i++)
	{
		va[i] = controller()->motionAtAbs(i).actualVel();
	}
	s_mm(6, 1, 6, fwd.Jf(), va, dX);

    ////////////////////////////////////////////////////step 1. compute vt(ee target velocity, pos loop)//////////////////////////////////////////////////////////////////////////////////////////////////
    double vt[6] = { 0 };
    double KPP[6] = {50,50,50,10,10,10};

    // linear vt //
	for (int i = 0; i < 3; ++i)
	{
        vt[i] = KPP[i] * (PqEnd0[i]- PqEnd[i]);
	}

    // angular vt //
    double dqt[4]={0};
    q2dqt(PqEnd, PqEnd0, dqt, KPP[4]);
    aris::dynamic::s_wq2wa(PqEnd + 3, dqt, vt+3);

    // protect vt max and min //
    const double vt_max[6]{0.2,0.2,0.2,0.1,0.1,0.1};
    const double vt_min[6]{-0.2,-0.2,-0.2,-0.1,-0.1,-0.1};
    for (int i = 0; i < 6; ++i)
    {
        vt[i] = std::max(vt_min[i], vt[i]);  // protect vt
        vt[i] = std::min(vt_max[i], vt[i]);  // protect vt
    }

    // force control vt //
    ////////////////////////////////////////////////////step 2. compute at(ee target acceleration, vel loop)//////////////////////////////////////////////////////////////////////////////////////////////////
    double at[6] = { 0 },ft[6] = { 0 };

    const double KPV[6] = {100, 100, 100, 200, 200, 200};
    const double KIV[6] = {1000, 1000, 1000, 10000, 10000, 10000};

    static double ErrSumVt[7] = { 0 };
    const double err_sum_max[6]{0.2, 0.2, 0.2, 0.5, 0.5, 0.5};
    const double err_sum_min[6]{-0.2, -0.2, -0.2, -0.5, -0.5, -0.5};

    // acc control //
	for (int i = 0; i < 6; ++i)
	{
		ErrSumVt[i] = ErrSumVt[i] + (vt[i]-dX[i])*0.001;
        ErrSumVt[i] = std::min(ErrSumVt[i], err_sum_max[i]);  // protect integral value
        ErrSumVt[i] = std::max(ErrSumVt[i], err_sum_min[i]);  // protect integral value
        at[i] = KPV[i] * (vt[i]-dX[i]) + KIV[i]*ErrSumVt[i];
	}

    // fce control //
    double vt_motion_max = 0.05;
    static double err_sum_fce_vt = 0.0;

    if(count() < 1000) vt_motion_max = 0.05;
    else vt_motion_max = 0.5;


    // protect max and min velocity //
    const int motion = 2;
   if(dX[motion] > vt_motion_max)
    {
        err_sum_fce_vt += (vt_motion_max - dX[motion]) * 0.001;
        at[motion] = KPV[motion] * (vt_motion_max-dX[motion]) + KIV[motion] * err_sum_fce_vt;
    }
    else if(dX[motion] < -vt_motion_max)
    {
        err_sum_fce_vt += (-vt_motion_max - dX[motion]) * 0.001;
        at[motion] = KPV[motion] * (-vt_motion_max-dX[motion]) + KIV[motion] * err_sum_fce_vt;
    }
    else
    {
        if(err_sum_fce_vt < 0.0)
        {
            err_sum_fce_vt += (vt_motion_max - dX[motion]) * 0.001;
            err_sum_fce_vt = std::min(0.0, err_sum_fce_vt);
        }
        else
        {
            err_sum_fce_vt += (-vt_motion_max - dX[motion]) * 0.001;
            err_sum_fce_vt = std::max(0.0, err_sum_fce_vt);
        }

        at[motion] = KIV[motion] * err_sum_fce_vt;
    }

    ////////////////////////////////////////////////////step 3. compute ft(ee target force, acc 2 fce)//////////////////////////////////////////////////////////////////////////////////////////////////
    // inertia matrix //
    double A[36]={0};
    f(model(), A);
    s_mm(6, 1, 6, A, at,ft);

    // fce control //
    static double SumdX=0, SumFt=0;

    if(count()==1)
    {
        SumdX=0;
        SumFt=0;
    }

    double Vmin=-0.02;
    const double KPF=5, KIF=5, vis = 500;
    //const double KPF=15, KIF=5, vis = 1000;
    double target_f=10;
    SumdX=SumdX+(Vmin-dX[motion])*0.001;
    SumFt = SumFt+(target_f - stateTor1[motion])*0.001;
    ft[motion]-=KPF * (target_f - stateTor1[motion]) + KIF*SumFt + vis*dX[motion];
    //ft[motion]-=KPF * (target_f - stateTor1[motion])+KIF*SumFt+vis*dX[motion];
    //ft[motion]=KPF * (target_f - stateTor1[motion])+KIF*SumFt+0*dXKI*SumdX+vis*dX[motion];
    //ft[motion]=-ft[motion]+0*dXKI*SumdX;

/*
    //Y-Direction
    static double SumdX=0, SumFt=0;
    double Vmin=-0.02,dXKI=2000,vis=000;
    double KPF=5,KIF=5;
    double target_f=-10;

    SumdX=SumdX+(Vmin-dX[motion])*0.001;
    SumFt = SumFt+(target_f - stateTor1[motion])*0.001;
    ft[motion]-=KPF * (target_f - stateTor1[motion])+KIF*SumFt+vis*dX[motion];
    //ft[motion]=-ft[motion]+0*dXKI*SumdX;
*/
    ////////////////////////////////////////////////////step 4. compute tau(caused by ft)//////////////////////////////////////////////////////////////////////////////////////////////////
	double f2c_index[6] = { 9.07327526291993, 9.07327526291993, 17.5690184835913, 39.0310903520972, 66.3992503259041, 107.566785527965 };

    double f_static[6] = { 9,9,5,3,2,2 };
	double f_vel_JRC[6] = { 0,0,0,0,0,0 };

	double JoinTau[6] = { 0 };
	s_mm(6, 1, 6, fwd.Jf(), T(6), ft, 1, JoinTau, 1);

    ////////////////////////////////////////////////////step 5. dynamic compensation//////////////////////////////////////////////////////////////////////////////////////////////////
	for (int i = 0; i < 6; i++)
	{
        JoinTau[i] = JoinTau[i] + model()->motionPool()[i].mfDyn() + f_vel_JRC[i] * va[i] + 0 * f_static[i] * signV(va[i]);
        JoinTau[i] = JoinTau[i] * f2c_index[i];
	}


    ////////////////////////////////////////////////////others//////////////////////////////////////////////////////////////////////////////////////////////////
    for (int i = 0; i < 6; i++)
    {
        JoinTau[i] = std::max(-500.0, JoinTau[i]);
        JoinTau[i] = std::min(500.0, JoinTau[i]);

        controller()->motionAtAbs(i).setTargetToq(JoinTau[i]);
    }

    for (int i = 0; i < 6; i++)
        stateTor0[i] = stateTor1[i];

    if (count() % 300 == 0)
    {
        double err=(PqEnd0[2] - PqEnd[2]);
        cout<<step_pjs[1]<<"****"<<stateTor1[motion]<<"****"<<dX[motion]<<"****"<<at[motion]<<"****"<<err_sum_fce_vt<<std::endl;
    }

    lout << count() << ","
         << step_pjs[1] << ","
         << PqEnd[1] << ","
         << ft[motion] << ","
         << dX[motion] << ","
         << step_pjs[1] << ","
         << std::endl;

    bool ds_is_all_finished{ true };
    if (!enable_FCPressL)
        for (int i = 0; i < 6; ++i)
        {
            auto ret = controller()->motionPool().at(i).disable();
            if (ret)
            {
                ds_is_all_finished = false;
            }
        }

    //将目标电机由电流模式切换到位置模式
    if (count()==28000)
    {
        for (int i = 0; i < 6; ++i)
        {

                auto &cm = controller()->motionPool().at(i);
                controller()->motionPool().at(i).setModeOfOperation(8);
                auto ret = cm.mode(8);
                cm.setTargetPos(cm.actualPos());

        }
    }

    return 28000 - count();
}
auto ForceDirect::collectNrt()->void
{
    ReSetLimit(this);
}
ForceDirect::ForceDirect(const std::string &name) :Plan(name)
{

	command().loadXmlStr(
        "<Command name=\"FCPressL\">"
		"	<GroupParam>"
		"       <Param name=\"PressF\" default=\"0\"/>"
        "		<Param name=\"SensorType\"default=\"-20.0\"/>"
		"   </GroupParam>"
		"</Command>");

}


static std::atomic_bool enable_mvJoint = true;
struct MoveJointParam
{
    double PressF;
    double SensorType;
};
auto MoveJoint::prepareNrt()->void
{
    MoveJointParam param;
    for (auto &p : cmdParams())
    {
        if (p.first == "PressF")
            param.PressF = doubleParam(p.first);
        if (p.first == "SensorType")
            param.SensorType = doubleParam(p.first);

    }

    this->param() = param;
    ret() = std::vector<std::pair<std::string, std::any>>();

    for (auto &option : motorOptions()) option |=
        Plan::USE_TARGET_POS |
        Plan::NOT_CHECK_VEL_CONTINUOUS;


    enable_mvJoint=true;

    //读取动力学参数
    auto mat0 = dynamic_cast<aris::dynamic::MatrixVariable*>(&*model()->variablePool().findByName("estParasFT"));
    for (int i = 0;i < 16;i++)
		sixDistalMatrix.estParasFT[i] = mat0->data().data()[i];

    auto mat1 = dynamic_cast<aris::dynamic::MatrixVariable*>(&*model()->variablePool().findByName("estParasJoint"));
    for (int i = 0;i < 60 + 12;i++)
		JointMatrixFT.estParasJoint[i] = mat1->data().data()[i];

    //auto mat3 = dynamic_cast<aris::dynamic::MatrixVariable*>(&*model()->variablePool().findByName("LoadParas"));
    for (int i = 0;i < 10;i++)
		JointMatrixFT.LoadParas[i] = 0;//1 * mat3->data().data()[i];

}
auto MoveJoint::executeRT()->int
{
    auto &param = std::any_cast<MoveJointParam&>(this->param());

    static double step_pjs[6],begin_pjs[6];
    static double FT0[6];

    // 打印电流 //
    auto &cout = controller()->mout();
    // log 电流 //
    auto &lout = controller()->lout();

    // 获取当前起始点位置 //
    if (count() == 1)
    {
        SetLimit(this,4.0);
        for (int i = 0; i < 6; ++i)
        {
            step_pjs[i] = model()->motionPool()[i].mp();
            begin_pjs[i] = model()->motionPool()[i].mp();
            controller()->motionPool().at(i).setModeOfOperation(10);	//切换到电流控制
        }
        controller()->logFileRawName("motion_replay");
    }

    if (model()->solverPool().at(1).kinPos())return -1;

    double dTheta[6] = { 0 };

    double FT[6],FTemp[6];
    if(param.SensorType>0)
        GetATI(this,FT);
    else
        GetYuLi(this,FT);

    double pa[6],va[6],aa[6],ta[6],idealToq[6],idealFT[6];

    for (int i = 0; i < 6; i++)
    {
        pa[i] = controller()->motionAtAbs(i).actualPos();
        va[i] = controller()->motionAtAbs(i).actualVel();
        aa[i] = 0;
        ta[i] = controller()->motionAtAbs(i).actualToq() / f2c_index[i];
    }

    double Acv[12] = {0.8,0.0,0.8,0.0,0.8,0.0,0.5,0.0,0.5,0.0,0.5,0.0};

    JointMatrixFT.JointDragYang(pa, va, aa, ta, JointMatrixFT.estParasJoint, JointMatrixFT.LoadParas, idealToq, Acv);

    sixDistalMatrix.sixDistalCollision(pa, va, aa, FT, sixDistalMatrix.estParasFT, idealFT);
    for (int j = 0; j < 6; j++)
    {
        FTemp[j]=FT[j];
        FT[j]=FT[j]-idealFT[j];
    }

    if (count() == 1)
    {
        for (int j = 0; j < 6; j++)
        {
            FT0[j] = FT[j];
        }
    }

    double FT_KAI[6];
    for (int i = 0; i < 6; i++)
    {
        FT_KAI[i] = FT[i] - FT0[i];//In KAI Coordinate
    }

    double zero_check[6] = { 0.2,0.2,0.2,0.05,0.05,0.05 };
    for (int i = 0; i < 6; i++)
    {
        if (FT_KAI[i] < zero_check[i] && FT_KAI[i]>0)
            FT_KAI[i] = 1 / zero_check[i] * FT_KAI[i] * FT_KAI[i];//In KAI Coordinate
        else if (FT_KAI[i]<0 && FT_KAI[i]>-zero_check[i])
            FT_KAI[i] = -1 / zero_check[i] * FT_KAI[i] * FT_KAI[i];//In KAI Coordinate
    }

    double FmInWorld[6];
    FT2World(this, FT_KAI, FmInWorld);


    ///* Using Jacobian, TransMatrix from ARIS
    auto &fwd = dynamic_cast<aris::dynamic::ForwardKinematicSolver&>(model()->solverPool()[1]);
    fwd.cptJacobiWrtEE();
    //FmInWorld[2] = 0; FmInWorld[3] = 0; FmInWorld[4] = 0; FmInWorld[5] = 0;
    double JoinTau[6] = { 0 };
    s_mm(6, 1, 6,fwd.Jf() , T(6), FmInWorld, 1, JoinTau, 1);


    double ft_offset[6] = { 0 };
    double f2c_index[6] = { 9.07327526291993, 9.07327526291993, 17.5690184835913, 39.0310903520972, 66.3992503259041, 107.566785527965 };
    double f_static[6] = { 9,9,5,3,2,2 };
    double f_vel_JRC[6] = { 10,10,10,10,10,10 };
    double ExternTau[6] = { 0 };

/*
    //动力学
    for (int i = 0; i < 6; ++i)
    {
        model()->motionPool()[i].setMp(controller()->motionPool()[i].actualPos());
        model()->motionPool().at(i).setMv(controller()->motionAtAbs(i).actualVel());
        model()->motionPool().at(i).setMa(0.0);
    }

    model()->solverPool()[1].kinPos();
    model()->solverPool()[1].kinVel();
    model()->solverPool()[2].dynAccAndFce();
*/

    for (int i = 0; i < 6; i++)
    {
        //ExternTau[i] = ta[i] -JoinTau[i]- model()->motionPool()[i].mfDyn() - f_vel_JRC[i] * va[i] - 1 * f_static[i] * signV(va[i]);
        ExternTau[i] = ta[i] -JoinTau[i]- idealToq[i];
        ExternTau[i] = -ExternTau[i];
    }

    double rate=0.05;
    dTheta[0] = JoinTau[0] / 300/rate + ExternTau[0] / 1000/rate;
    dTheta[1] = JoinTau[1] / 500/rate + ExternTau[1] / 1000/rate;
    dTheta[2] = JoinTau[2] / 500/rate + ExternTau[2] / 1000/rate;
    dTheta[3] = JoinTau[3] / 200/rate + ExternTau[3] / 1000/rate;
    dTheta[4] = JoinTau[4] / 300/rate + ExternTau[4] / 1000/rate;
    dTheta[5] = JoinTau[5] / 300/rate + ExternTau[5] / 1000/rate;

    for (int i = 0; i < 6; i++)
    {
        if (dTheta[i] > 0.0006)
            dTheta[i] = 0.0006;
        if (dTheta[i] < -0.0006)
            dTheta[i] = -0.0006;
        //lout << dTheta[i] << ",";
    }

    for (int i = 0; i < 6; i++)
    {
        step_pjs[i] = step_pjs[i] + dTheta[i];
       // model()->motionPool().at(i).setMp(step_pjs[i]);
    }

    double KP[6]={8,10,10,1,2,0.1};

    double torque_max[6]={300,500,500,300,300,400};
    double torque_min[6]={-300,-500,-500,-300,-300,-400};
    for (int i = 0; i < 6; i++)
    {

     ft_offset[i]=(30*KP[i]*(step_pjs[i]-pa[i])+idealToq[i])*f2c_index[i];
     ft_offset[i] = std::max(torque_min[i], ft_offset[i]);
     ft_offset[i] = std::min(torque_max[i], ft_offset[i]);
     controller()->motionAtAbs(i).setTargetToq(ft_offset[i]);
     
    }

    lout << pa[0] << " ";lout << pa[1] << " ";
    lout << pa[2] << " ";lout << pa[3] << " ";
    lout << pa[4] << " ";lout << pa[5] << " ";
    lout << std::endl;

    if (count() % 300 == 0)
    {
        cout<<ft_offset[0]<<"***"<< ft_offset[1]<<"***"<< ft_offset[2]<<"***"<<ft_offset[3]<<"***"<<ft_offset[4]<<std::endl;
    }

    if (!enable_mvJoint)
        for (int i = 0; i < 6; ++i)
            auto ret = controller()->motionPool().at(i).disable();
    return 150000000 - count();
}
auto MoveJoint::collectNrt()->void
{
    ReSetLimit(this);
}
MoveJoint::MoveJoint(const std::string &name) :Plan(name)
{

    command().loadXmlStr(
        "<Command name=\"mvJoint\">"
        "	<GroupParam>"
        "       <Param name=\"PressF\" default=\"0\"/>"
        "		<Param name=\"SensorType\"default=\"-20.0\"/>"
        "   </GroupParam>"
        "</Command>");

}


//复现文件位置//
struct ReplayParam
{
    std::vector<aris::Size> total_count_vec;
    std::vector<double> axis_begin_pos_vec;
    std::vector<double> axis_first_pos_vec;
    std::vector<std::vector<double>> pos_vec;
    double vel, acc, dec;
    std::string path;
};
auto Replay::prepareNrt()->void
{
    ReplayParam param;
    param.vel = doubleParam("vel");
    param.acc = doubleParam("acc");
    param.dec = doubleParam("dec");
    param.path = cmdParams().at("path");

    param.total_count_vec.resize(6, 0);
    param.axis_begin_pos_vec.resize(6, 0.0);
    param.axis_first_pos_vec.resize(6, 0.0);

    param.pos_vec.resize(6, std::vector<double>(1, 0.0));
    //std::cout << "size:" << param.pos_vec.size() << std::endl;
    //初始化pos_vec//
    for (int j = 0; j < param.pos_vec.size(); j++)
    {
        param.pos_vec[j].clear();
    }

    //定义读取log文件的输入流oplog//
    std::ifstream oplog;
    int cal = 0;
    oplog.open(param.path);

    //以下检查是否成功读取文件//
    if (!oplog)
    {
        throw std::runtime_error("fail to open the file");
    }
    while (!oplog.eof())
    {
        for (int j = 0; j < param.pos_vec.size(); j++)
        {
            double data;
            oplog >> data;
            param.pos_vec[j].push_back(data);
        }
    }
    oplog.close();
    //oplog.clear();
    for (int j = 0; j < param.pos_vec.size(); j++)
    {
        param.pos_vec[j].pop_back();
        param.axis_first_pos_vec[j] = param.pos_vec[j][0];
    }

	this->param() = param;
    std::fill(motorOptions().begin(), motorOptions().end(),
        Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER|
        Plan::NOT_CHECK_POS_FOLLOWING_ERROR);
    std::vector<std::pair<std::string, std::any>> ret_value;
    ret() = ret_value;
}
auto Replay::executeRT()->int
{
    auto &param = std::any_cast<ReplayParam&>(this->param());

    double p, v, a;
    aris::Size t_count;
    static aris::Size first_total_count = 1;
    aris::Size total_count = 1;
    aris::Size return_value = 0;

    // 获取6个电机初始位置 //
    if (count() == 1)
    {
        for (int i = 0; i < 6; i++)
        {
            param.axis_begin_pos_vec[i] = model()->motionPool().at(i).mp();
        }
        for (int i = 0; i < 6; i++)
        {
            // 梯形规划到log开始点 //
            aris::plan::moveAbsolute(count(), param.axis_begin_pos_vec[i], param.axis_first_pos_vec[i], param.vel / 1000, param.acc / 1000 / 1000, param.dec / 1000 / 1000, p, v, a, t_count);
            first_total_count = std::max(first_total_count, t_count);
        }
    }

    // 机械臂走到log开始点 //
    if (count() <= first_total_count)
    {
        for (int i = 0; i < 6; i++)
        {
            // 在第一个周期走梯形规划复位
            aris::plan::moveAbsolute(count(), param.axis_begin_pos_vec[i], param.axis_first_pos_vec[i], param.vel / 1000, param.acc / 1000 / 1000, param.dec / 1000 / 1000, p, v, a, t_count);
            controller()->motionAtAbs(i).setTargetPos(p);
            model()->motionPool().at(i).setMp(p);
        }
    }

    // 机械臂开始从头到尾复现log中点 //
    if (count() > first_total_count)
    {
        for (int i = 0; i < 6; i++)
        {
            controller()->motionAtAbs(i).setTargetPos(param.pos_vec[i][count() - first_total_count]);
            model()->motionPool().at(i).setMp(param.pos_vec[i][count() - first_total_count]);
        }
    }
    if (model()->solverPool().at(1).kinPos())return -1;

    //输出6个轴的实时位置log文件//
    auto &lout = controller()->lout();
    for (int i = 0; i < 6; i++)
    {
        lout << controller()->motionAtAbs(i).actualPos() << " ";//第一列数字必须是位置
    }
    lout << std::endl;

    return count() > (first_total_count - 2 + param.pos_vec[0].size()) ? 0 : 1;

}
auto Replay::collectNrt()->void {}
Replay::Replay(const std::string &name) :Plan(name)
{
    command().loadXmlStr(
        "<Command name=\"MotionReplay\">"
        "	<GroupParam>"
        "		<Param name=\"path\" default=\"C:\\Users\\kevin\\Desktop\\file\\rt_log--2019-09-06--19-14-16--moveJR.txt\"/>"
        "		<Param name=\"vel\" default=\"0.05\" abbreviation=\"v\"/>"
        "		<Param name=\"acc\" default=\"0.1\" abbreviation=\"a\"/>"
        "		<Param name=\"dec\" default=\"0.1\" abbreviation=\"d\"/>"
        "	</GroupParam>"
        "</Command>");
}


static std::atomic_bool enable_FCPressP = true;
// 力控停止指令——停止FCStop，去使能电机 //
auto FCStop::prepareNrt()->void
    {
        enable_mvJoint = false;
        enable_FCPressL=false;
        enable_FCPressP=false;
		option() = aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION;

    }
FCStop::FCStop(const std::string &name) :Plan(name)
    {
        command().loadXmlStr(
            "<Command name=\"FCStop\">"
            "</Command>");
    }
