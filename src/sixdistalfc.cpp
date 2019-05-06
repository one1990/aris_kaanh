﻿#include "sixdistalfc.h"
#include <math.h>
#include"kaanh.h"
#include <algorithm>
#include"robotconfig.h"
#include"sixdistaldynamics.h"
#include <vector>
using namespace std;
using namespace aris::plan;
using namespace aris::dynamic;
using namespace CONFIG;
using namespace sixDistalDynamicsInt;
/// \brief

robotconfig robotDemo;
sixdistaldynamics sixDistalMatrix;
double estParas[GroupDim]={0};
double StatisError[6]={0,0,0,0,0,0};

std::vector<double> PositionList_vec(6 * SampleNum);
auto PositionList = PositionList_vec.data();
std::vector<double> SensorList_vec(6 * SampleNum);
auto SensorList = SensorList_vec.data();

double ForceToMeng=0;
double TimeToMeng = 0;

struct MoveXYZParam
{
	double damp[6];

};
auto MoveXYZ::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
{
    MoveXYZParam param;


	for (auto &p : params)
	{

		if (p.first == "damp")
		{
			for (int i = 0; i < 6; i++)
			   param.damp[i] = std::stod(p.second);
		}
	
	}
	target.param = param;

	target.option |=
		Plan::USE_TARGET_POS |
#ifdef WIN32
		Plan::NOT_CHECK_POS_MIN |
		Plan::NOT_CHECK_POS_MAX |
		Plan::NOT_CHECK_POS_CONTINUOUS |
		Plan::NOT_CHECK_POS_CONTINUOUS_AT_START |
        Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
		Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER_AT_START |
		Plan::NOT_CHECK_POS_FOLLOWING_ERROR |
#endif
            Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
            Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER_AT_START |
		Plan::NOT_CHECK_VEL_MIN |
		Plan::NOT_CHECK_VEL_MAX |
		Plan::NOT_CHECK_VEL_CONTINUOUS |
		Plan::NOT_CHECK_VEL_CONTINUOUS_AT_START |
		Plan::NOT_CHECK_VEL_FOLLOWING_ERROR;
	
   
}


void crossVector(const double* a, const double* b, double* c)
{
	
	c[0] = a[1] * b[2] - b[1] * a[2];
	c[1] = -(a[0] * b[2] - b[0] * a[2]);
	c[2] = a[0] * b[1] - b[0] * a[1];

}

auto MoveXYZ::executeRT(PlanTarget &target)->int
{
			auto &param = std::any_cast<MoveXYZParam&>(target.param);
			
			double RobotPosition[6];
			double RobotPositionJ[6];
			double RobotVelocity[6];
			double RobotAcceleration[6];
			double TorqueSensor[6];
			double X1[3];
			double X2[3];
			static double begin_pjs[6];
			static double step_pjs[6];
            static double stateTor0[6][3],stateTor1[6][3];
            static float FT0[6],FT_be[6];

            // 访问主站 //
            auto controller = target.controller;

				// 获取当前起始点位置 //
				if (target.count == 1)
				{
                    for (int i = 0; i < 6; ++i)
					{
						step_pjs[i] = target.model->motionPool()[i].mp();
                       // controller->motionPool().at(i).setModeOfOperation(10);	//切换到电流控制
                    }
				}
				

			if (!target.model->solverPool().at(1).kinPos())return -1;


			///* Using Jacobian, TransMatrix from ARIS
			double EndW[3], EndP[3], BaseV[3];
			double PqEnd[7], TransVector[16];
			target.model->generalMotionPool().at(0).getMpm(TransVector);
            target.model->generalMotionPool().at(0).getMpq(PqEnd);

            double dX[6] = { 0.00001, -0.0000, -0.0000, -0.0000, -0.0000, -0.0000};
            double dTheta[6]={0};

            float FT[6];
            uint16_t FTnum;
            auto conSensor = dynamic_cast<aris::control::EthercatController*>(target.controller);
            conSensor->ecSlavePool().at(6).readPdo(0x6030, 0x00, &FTnum ,16);
            conSensor->ecSlavePool().at(6).readPdo(0x6030, 0x01, &FT[0] ,32);  //Fx
            conSensor->ecSlavePool().at(6).readPdo(0x6030, 0x02, &FT[1], 32);  //Fy
            conSensor->ecSlavePool().at(6).readPdo(0x6030, 0x03, &FT[2], 32);  //Fz
            conSensor->ecSlavePool().at(6).readPdo(0x6030, 0x04, &FT[3], 32);
            conSensor->ecSlavePool().at(6).readPdo(0x6030, 0x05, &FT[4], 32);
            conSensor->ecSlavePool().at(6).readPdo(0x6030, 0x06, &FT[5], 32);

            FT[0]=-FT[0];FT[3]=-FT[3];


            // 获取当前起始点位置 //
            if (target.count == 1)
            {
                for (int j = 0; j < 6; j++)
                {
                    stateTor0[j][0] = FT[j];
                    FT0[j] = FT[j];
                    FT_be[j]=FT[j];
                }
            }

            for (int j = 0; j < 6; j++)
            {
                if(abs(FT[j])<0.0001)
                   FT[j]=FT_be[j];
            }


            for (int j = 0; j < 6; j++)
            {
                double A[3][3], B[3], CutFreq = 5;
                A[0][0] = 0; A[0][1] = 1; A[0][2] = 0;
                A[1][0] = 0; A[1][1] = 0; A[1][2] = 1;
                A[2][0] = -CutFreq * CutFreq * CutFreq;
                A[2][1] = -2 * CutFreq * CutFreq;
                A[2][2] = -2 * CutFreq;
                B[0] = 0; B[1] = 0;
                B[2] = -A[2][0];
                double intDT = 0.001;
                stateTor1[j][0] = stateTor0[j][0] + intDT * (A[0][0] * stateTor0[j][0] + A[0][1] * stateTor0[j][1] + A[0][2] * stateTor0[j][2] + B[0] * FT[j]);
                stateTor1[j][1] = stateTor0[j][1] + intDT * (A[1][0] * stateTor0[j][0] + A[1][1] * stateTor0[j][1] + A[1][2] * stateTor0[j][2] + B[1] * FT[j]);
                stateTor1[j][2] = stateTor0[j][2] + intDT * (A[2][0] * stateTor0[j][0] + A[2][1] * stateTor0[j][1] + A[2][2] * stateTor0[j][2] + B[2] * FT[j]);
            }


            for (int i = 0;i < 6;i++)
            {
                RobotPositionJ[i] = target.model->motionPool()[i].mp();
                RobotPosition[i] = target.model->motionPool()[i].mp();
                RobotVelocity[i]=0;
                RobotAcceleration[i]=0;
                TorqueSensor[i]=stateTor1[i][0];
            }
            double estFT[6]={0};
            sixDistalMatrix.sixDistalCollision(RobotPosition, RobotVelocity, RobotAcceleration, TorqueSensor, estParas, estFT);

            double FT_KAI[6];
            for (int i = 0; i < 6; i++)
            {
                    FT_KAI[i]=stateTor1[i][0]-estFT[i];
            }



            for (int i = 0; i < 3; i++)
            {
                if (abs(FT_KAI[i]) < 2)
                      FT_KAI[i]=0;
            }
            for (int i = 3; i < 6; i++)
            {
                if (abs(FT_KAI[i]) < 0.05)
                      FT_KAI[i]=0;
            }

            double FT_YANG[6];
            FT_YANG[0]=FT_KAI[2];FT_YANG[1]=-FT_KAI[1];FT_YANG[2]=FT_KAI[0];
            FT_YANG[3]=FT_KAI[5];FT_YANG[4]=-FT_KAI[4];FT_YANG[5]=FT_KAI[3];

			double FmInWorld[6];

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



               dX[0] = 1*FmInWorld[0] / 80000;
               dX[1] = 1*FmInWorld[1] / 80000;
               dX[2] = 1*FmInWorld[2] / 80000;
               dX[3] = 1*FmInWorld[3] / 12000;
               dX[4] = 1*FmInWorld[4] / 12000;
               dX[5] = 1*FmInWorld[5] / 12000;


               if (target.count % 100 == 0)
               {
                   for (int i = 0; i < 6; i++)
                   {
                       cout<<dX[i]<<"***"<<FT[i]<<"***"<<FmInWorld[i]<<endl;

                   }

                   cout << std::endl;

               }


			   for (int j = 0; j < 6; j++)
			   {
                   if (dX[j] > 0.00025)
                       dX[j] = 0.00025;
                   if (dX[j] < -0.00025)
                       dX[j] = -0.00025;
			   }


               // 打印电流 //
               auto &cout = controller->mout();





               // log 电流 //
               auto &lout = controller->lout();

              // lout << target.model->motionPool()[0].mp() << ",";
               //lout << target.model->motionPool()[1].mp() << ",";
               //lout << target.model->motionPool()[2].mp() << ",";
               //lout << target.model->motionPool()[3].mp() << ",";
               //lout << target.model->motionPool()[4].mp() << ",";
               //lout << target.model->motionPool()[5].mp() << ",";


               lout << FT[1] << ",";lout << FT[2] << ",";
               lout << FT[3] << ",";lout << FT[4] << ",";
               lout << FT[5] << ",";lout << FT[6] << ",";
               lout << stateTor0[0][0] << ",";lout << stateTor0[1][0] << ",";
               lout << stateTor0[2][0] << ",";lout << stateTor0[3][0] << ",";
               lout << stateTor0[4][0] << ",";lout << stateTor0[5][0] << ",";

               //lout << stateTor1[2][0] << ",";lout << FT0[3] << ",";
              // lout << dX[0] << ",";

              // lout << dX[0] << ",";
              // lout << FT[1] << ",";lout << FT[2] << ",";
              // lout << FT[3] << ",";lout << FT[4] << ",";
              // lout << FT[5] << ",";lout << FT[6] << ",";
               lout << std::endl; 



               ///* Using Jacobian, TransMatrix from ARIS
		       for (int i = 0;i < 3;i++)
				   EndW[i] = dX[i+3];

			   for (int i = 0;i < 3;i++)
				   EndP[i] = PqEnd[i];
			   crossVector(EndP, EndW, BaseV);

			   for (int i = 0;i < 3;i++)
				   dX[i + 3] = dX[i + 3];
			   for (int i = 0;i < 3;i++)
				   dX[i] = dX[i]+ BaseV[i];


				auto &fwd = dynamic_cast<aris::dynamic::ForwardKinematicSolver&>(target.model->solverPool()[1]);
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
						U[7 * i] = U[7 * i] + 0.1;
					else
						U[7 * i] = U[7 * i] - 0.1;
				// 根据QR分解的结果求x，相当于Matlab中的 x = A\b //
				s_householder_utp_sov(6, 6, 1, rank, U, tau, p, dX, dTheta, 1e-10);

				// 根据QR分解的结果求广义逆，相当于Matlab中的 pinv(A) //
				double tau2[6];
				s_householder_utp2pinv(6, 6, rank, U, tau, p, pinv, tau2, 1e-10);

				// 根据QR分解的结果求广义逆，相当于Matlab中的 pinv(A)*b //
				s_mm(6, 1, 6, pinv, dX, dTheta);



                for (int i = 0; i < 6; i++)
                {
                    if (dTheta[i] > 0.003)
                        dTheta[i] = 0.003;
                    if (dTheta[i] < -0.003)
                       dTheta[i] = -0.003;
                     //lout << dTheta[i] << ",";
                }


                //lout << std::endl;

				for (int i = 0; i < 6; i++)
				{
					step_pjs[i] = step_pjs[i] + dTheta[i];
                    target.model->motionPool().at(i).setMp(step_pjs[i]);
				}

                for (int i = 0; i < 6; i++)
                {
                    dTheta[i] = dTheta[i] * DirectionFlag[i];
 
                }
                     
               
                for (int i = 0; i < 6; i++)
                {

                    stateTor0[i][0] = stateTor1[i][0];
                    stateTor0[i][1] = stateTor1[i][1];
                    stateTor0[i][2] = stateTor1[i][2];
                }

				/*
                for (int i = 0; i < 6; i++)
                {
                    stateDm0[i, 0] = stateDm1[i, 0];
                    stateDm0[i, 1] = stateDm1[i, 1];
                    stateDm0[i, 2] = stateDm1[i, 2];

                    stateTor0[i, 0] = stateTor1[i, 0];
                    stateTor0[i, 1] = stateTor1[i, 1];
                    stateTor0[i, 2] = stateTor1[i, 2];
                } */
            
                for (int j = 0; j < 6; j++)
                {
                    FT_be[j]=FT[j];
                }
                return 150000000 - target.count;
}

MoveXYZ::MoveXYZ(const std::string &name) :Plan(name)
    {
        command().loadXmlStr(
            "<Command name=\"mvXYZ\">"
			"	<GroupParam>"
			"		<Param name=\"damp\" default=\"1.0\"/>"
			"	</GroupParam>"
            "</Command>");


    }



struct MoveDistalParam
{
	double period;
	double amplitude;
	
};
auto MoveDistal::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
{
	MoveDistalParam param;

	for (auto &p : params)
	{
		if (p.first == "period")
		    param.period = std::stod(p.second);
		if (p.first == "amplitude")
			param.amplitude = std::stod(p.second);

	}

	target.param = param;

	target.option |=
		Plan::USE_TARGET_POS |
//#ifdef WIN32
		Plan::NOT_CHECK_POS_MIN |
		Plan::NOT_CHECK_POS_MAX |
		Plan::NOT_CHECK_POS_CONTINUOUS |
		Plan::NOT_CHECK_POS_CONTINUOUS_AT_START |
		Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
		Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER_AT_START |
		Plan::NOT_CHECK_POS_FOLLOWING_ERROR |
//#endif
		Plan::NOT_CHECK_VEL_MIN |
		Plan::NOT_CHECK_VEL_MAX |
		Plan::NOT_CHECK_VEL_CONTINUOUS |
		Plan::NOT_CHECK_VEL_CONTINUOUS_AT_START |
		Plan::NOT_CHECK_VEL_FOLLOWING_ERROR;


}
auto MoveDistal::executeRT(PlanTarget &target)->int
{
	auto &param = std::any_cast<MoveDistalParam&>(target.param);

	static double begin_pjs[6];
	static double step_pjs[6];
	static double perVar = 0;
	static double ampVar = 0;

	if (target.count < 1000)
	{
		ampVar = ampVar + param.amplitude / 1000;
	}
		// 获取当前起始点位置 //
	if (target.count == 1)
	{
			for (int i = 0; i < 6; i++)
			{
				begin_pjs[i] = target.model->motionPool()[i].mp();
				step_pjs[i] = target.model->motionPool()[i].mp();
			}
	}
   param.period=60;

   static bool flag4=true,flag5=true;
   double dTheta=0.0001;
            if(flag4)
            {
                if(step_pjs[4]<1.40)
                    step_pjs[4] = step_pjs[4] + dTheta;
                else
                    flag4=false;
            }
            if(flag4==false)
            {
                if(step_pjs[4]>-1.40)
                    step_pjs[4] = step_pjs[4] - dTheta;
                else
                    flag4=true;
            }


            target.model->motionPool().at(4).setMp(step_pjs[4]);

            if(flag5)
            {
                if(step_pjs[5]<1.40)
                    step_pjs[5] = step_pjs[5] + dTheta;
                else
                    flag5=false;
            }
            if(flag5==false)
            {
                if(step_pjs[5]>-1.40)
                    step_pjs[5] = step_pjs[5] - dTheta;
                else
                    flag5=true;
            }
            target.model->motionPool().at(5).setMp(step_pjs[5]);


	
    if (!target.model->solverPool().at(1).kinPos())return -1;

	// 访问主站 //
	auto controller = target.controller;

    float FT[6];
    int16_t FTnum;
    auto conSensor = dynamic_cast<aris::control::EthercatController*>(target.controller);
    conSensor->ecSlavePool().at(6).readPdo(0x6030, 0x00, &FTnum ,16);
    conSensor->ecSlavePool().at(6).readPdo(0x6030, 0x01, &FT[0] ,32);
    conSensor->ecSlavePool().at(6).readPdo(0x6030, 0x02, &FT[1], 32);
    conSensor->ecSlavePool().at(6).readPdo(0x6030, 0x03, &FT[2], 32);
    conSensor->ecSlavePool().at(6).readPdo(0x6030, 0x04, &FT[3], 32);
    conSensor->ecSlavePool().at(6).readPdo(0x6030, 0x05, &FT[4], 32);
    conSensor->ecSlavePool().at(6).readPdo(0x6030, 0x06, &FT[5], 32);
    FT[0]=-FT[0];FT[3]=-FT[3];

	// 打印电流 //
	auto &cout = controller->mout();
    if (target.count % 100 == 0)
	{
        //for (int i = 0; i < 6; i++)
        {
            cout <<step_pjs[4]<<"***"<<flag4<< "  ";
            //cout << "vel" << i + 1 << ":" << target.model->motionPool()[i].mv() << "  ";
            //cout << "cur" << i + 1 << ":" << target.model->motionPool()[i].ma() << "  ";
        }
     //   cout << target.count << "  ";
        cout << std::endl;
	}

    auto &lout = controller->lout();

		for (int i = 0; i < 6; i++)
		{
            PositionList[6*(target.count-1)+i] = target.model->motionPool()[i].mp();
            SensorList[6*(target.count-1)+i] = FT[i];
		}



		    lout << target.count << ",";
			lout << PositionList[6 * (target.count - 1) + 0] << ",";lout << PositionList[6 * (target.count - 1) + 1] << ",";
			lout << PositionList[6 * (target.count - 1) + 2] << ",";lout << PositionList[6 * (target.count - 1) + 3] << ",";
			lout << PositionList[6 * (target.count - 1) + 4] << ",";lout << PositionList[6 * (target.count - 1) + 5] << ",";
			lout << SensorList[6 * (target.count - 1) + 0] << ",";lout << SensorList[6 * (target.count - 1) + 1] << ",";
			lout << SensorList[6 * (target.count - 1) + 2] << ",";lout << SensorList[6 * (target.count - 1) + 3] << ",";
			lout << SensorList[6 * (target.count - 1) + 4] << ",";lout << SensorList[6 * (target.count - 1) + 5] << ",";

			lout << std::endl;


    return SampleNum-target.count;
}






auto MoveDistal::collectNrt(aris::plan::PlanTarget &target)->void
{

  //  auto controller = target.controller;
   // auto &lout = controller->lout();
    std::cout<<"collect"<<std::endl;

    sixDistalMatrix.RLS(PositionList, SensorList, estParas,StatisError);
//std::cout<<"collect"<<std::endl;
    for(int i=0;i<GroupDim;i++)
        cout<<estParas[i]<<",";
	
	//Save Estimated Paras
	ofstream outfile("C:/Users/gk/Desktop/Kaanh_gk/EstParas.txt");
	if (!outfile)
	{
		cout << "Unable to open otfile";
		exit(1); // terminate with error
	}

	for (int k = 0; k < GroupDim; k++)
	{
		outfile << estParas[k] << "   " << std::endl;
		//cout << "success outfile " << endl;
	}
	outfile.close();


    std::cout<<endl;
    std::cout<<"*****************************Statictic Model Error*****************************************"<<std::endl;
    for(int i=0;i<6;i++)
        cout<<StatisError[i]<<endl;
    //double a = 3;
}


MoveDistal::MoveDistal(const std::string &name) :Plan(name)
{
	
	command().loadXmlStr(
		"<Command name=\"mvDistal\">"
		"	<GroupParam>"
        "		<Param name=\"period\"default=\"20.0\"/>"
        "		<Param name=\"amplitude\" default=\"0.2\"/>"
		"	</GroupParam>"
		"</Command>");

}



struct SetToolParam
{
	double period;
	double amplitude;

};
auto SetTool::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
{
	SetToolParam param;

	for (auto &p : params)
	{
		if (p.first == "period")
			param.period = std::stod(p.second);
		if (p.first == "amplitude")
			param.amplitude = std::stod(p.second);

	}

	target.param = param;

	target.option |=
		Plan::USE_TARGET_POS |
		//#ifdef WIN32
		Plan::NOT_CHECK_POS_MIN |
		Plan::NOT_CHECK_POS_MAX |
		Plan::NOT_CHECK_POS_CONTINUOUS |
		Plan::NOT_CHECK_POS_CONTINUOUS_AT_START |
		Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
		Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER_AT_START |
		Plan::NOT_CHECK_POS_FOLLOWING_ERROR |
		//#endif
		Plan::NOT_CHECK_VEL_MIN |
		Plan::NOT_CHECK_VEL_MAX |
		Plan::NOT_CHECK_VEL_CONTINUOUS |
		Plan::NOT_CHECK_VEL_CONTINUOUS_AT_START |
		Plan::NOT_CHECK_VEL_FOLLOWING_ERROR;

	
	
}
auto SetTool::executeRT(PlanTarget &target)->int
{
	auto &param = std::any_cast<SetToolParam&>(target.param);


	double TransMatrix[6][12] = { { -0.6492,0.2770,0.7084,-0.2863,0.7738,-0.5650,-0.7046,-0.5697,-0.4231,1028.29,233.29,1432.98},
	{-0.8469, 0.1182, 0.5184, 0.4204, 0.7458, 0.5168, -0.3256, 0.6556, -0.6813,1118.69,-231.99,1478.85},
	{-0.8680, -0.0425, 0.4947, -0.0744, 0.9962, -0.0451, -0.4909, -0.0759, -0.8679,1129.45,25,1564},
	{-0.5835, 0.0035, 0.8121, 0.0020, 1.0000, -0.0028, -0.8121, 0.0000, -0.5835,983.98,0.39,1502.31},
	{ -0.5835, 0.0035, 0.8121, 0.0020, 1.0000, -0.0028, -0.8121, 0.0000, -0.5835,605.47,0.39,1502.31},
	{ -0.5835, 0.0035, 0.8121, 0.0020, 1.0000, -0.0028, -0.8121, 0.0000, -0.5835,983.98,0.39,1122.24}};
	double Atemp[5][9],Btemp[5][3];
	
	//target.model->generalMotionPool().at(0).getMpm(TransVector);
	for (int i = 0;i < 5;i++)
		for(int j=0;j<9;j++)
			Atemp[i][j] = TransMatrix[i][j] - TransMatrix[i+1][j];
	
	double L[3][3] = { 0 };
	for (int i = 0;i < 5;i++)
	{
		for (int m = 0;m < 3;m++)
			for (int n = 0;n < 3;n++)
				for (int j = 0;j < 3;j++)
				L[m][n] = L[m][n] + Atemp[i][j+3*m] * Atemp[i][j*3 + n];
	}

	for (int i = 0;i < 5;i++)
		for (int j = 0;j < 3;j++)
			Btemp[i][j] = TransMatrix[i + 1][j + 9] - TransMatrix[i][j + 9];

	double R[3] = { 0 };
	for (int i = 0;i < 5;i++)
	{
		for (int m = 0;m < 3;m++)
				for (int j = 0;j < 3;j++)
					R[m] = R[m] + Atemp[i][j + 3 * m] * Btemp[i][j];
	}

	// Pinv(L)*R,求取位置偏移Epos
	double Lvec[9];
	for(int i=0;i<3;i++)
		for (int j = 0;j < 3;j++)
			Lvec[i * 3 + j] = L[i][j];
		
	double U[9], tau[3], pinv[9], Epos[3];
	aris::Size p[3];
	aris::Size rank;

	// 根据 A 求出中间变量，相当于做 QR 分解 //
	// 请对 U 的对角线元素做处理
	s_householder_utp(3, 3, Lvec, U, tau, p, rank, 1e-10);
	
	// 根据QR分解的结果求广义逆，相当于Matlab中的 pinv(A) //
	double tau2[3];
	s_householder_utp2pinv(3, 3, rank, U, tau, p, pinv, tau2, 1e-10);

	// 根据QR分解的结果求广义逆，相当于Matlab中的 pinv(A)*b //
	s_mm(3, 1, 3, pinv, R, Epos);

	// 求取姿态偏移矩阵Epos
	double Trans4[9] = { -0.5835, 0.0035, 0.8121, 0.0020, 1.0000, -0.0028, -0.8121, 0.0000, -0.5835 };
	double Dis4[3] = { 983.98,0.39,1502.31 };
	double Dis5[3] = { 605.47,0.39,1502.31 };
	double Dis6[3] = { 983.98,0.39,1122.24 };
	double x1[3], x2[3],x12[3];
	s_mm(3, 1, 3, Trans4, Epos, x1);
	for (int i = 0;i < 3;i++)
		x1[i] = x1[i] + Dis4[i];
	s_mm(3, 1, 3, Trans4, Epos, x2);
	for (int i = 0;i < 3;i++)
		x2[i] = x2[i] + Dis5[i];

	//计算X方向的方向余弦
	double norm_x12 = 0,n_TE[3];
	for (int i = 0;i < 3;i++)
	{
        x12[i] = x2[i] - x1[i];
		norm_x12 = norm_x12 + x12[i] * x12[i];
	}
	norm_x12 = sqrt(norm_x12);
	for (int i = 0;i < 3;i++)
	{
		x12[i] = x12[i] / norm_x12;
	}
	// 根据 A 求出中间变量，相当于做 QR 分解 //
	s_householder_utp(3, 3, Trans4, U, tau, p, rank, 1e-10);

	// 根据QR分解的结果求广义逆，相当于Matlab中的 pinv(A) //
	s_householder_utp2pinv(3, 3, rank, U, tau, p, pinv, tau2, 1e-10);

	// 根据QR分解的结果求广义逆，相当于Matlab中的 pinv(A)*b //
	s_mm(3, 1, 3, pinv, x12, n_TE);

	//计算Z方向的方向余弦
	s_mm(3, 1, 3, Trans4, Epos, x2);
	for (int i = 0;i < 3;i++)
		x2[i] = x2[i] + Dis6[i];
	double a_TE[3];
	for (int i = 0;i < 3;i++)
	{
		x12[i] = x2[i] - x1[i];
		norm_x12 = norm_x12 + x12[i] * x12[i];
	}
	norm_x12 = sqrt(norm_x12);
	for (int i = 0;i < 3;i++)
	{
		x12[i] = x12[i] / norm_x12;
	}
	// 根据 A 求出中间变量，相当于做 QR 分解 //
	s_householder_utp(3, 3, Trans4, U, tau, p, rank, 1e-10);

	// 根据QR分解的结果求广义逆，相当于Matlab中的 pinv(A) //
	s_householder_utp2pinv(3, 3, rank, U, tau, p, pinv, tau2, 1e-10);

	// 根据QR分解的结果求广义逆，相当于Matlab中的 pinv(A)*b //
	s_mm(3, 1, 3, pinv, x12, a_TE);

	//计算Y方向的方向余弦
	double o_TE[3];
	crossVector(a_TE, n_TE, o_TE);
	return 10 - target.count;
}

auto SetTool::collectNrt(aris::plan::PlanTarget &target)->void
{

    //sixDistalMatrix.RLS(PositionList, SensorList, estParas);
	double a = 3;
}


SetTool::SetTool(const std::string &name) :Plan(name)
{
	
	command().loadXmlStr(
		"<Command name=\"STool\">"
		"	<GroupParam>"
		"		<Param name=\"period\"default=\"1.0\"/>"
		"		<Param name=\"amplitude\"default=\"0.2\"/>"
		"	</GroupParam>"
		"</Command>");
		

}

struct MovePressureParam
{
	double PressF;
	

};

auto MovePressure::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
{
	MovePressureParam param;
	for (auto &p : params)
	{
		if (p.first == "PressF")
			param.PressF = std::stod(p.second);

	}

	target.param = param;

	target.option |=
		Plan::USE_TARGET_POS |
		//#ifdef WIN32
		Plan::NOT_CHECK_POS_MIN |
		Plan::NOT_CHECK_POS_MAX |
		Plan::NOT_CHECK_POS_CONTINUOUS |
		Plan::NOT_CHECK_POS_CONTINUOUS_AT_START |
		Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
		Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER_AT_START |
		Plan::NOT_CHECK_POS_FOLLOWING_ERROR |
		//#endif
		Plan::NOT_CHECK_VEL_MIN |
		Plan::NOT_CHECK_VEL_MAX |
		Plan::NOT_CHECK_VEL_CONTINUOUS |
		Plan::NOT_CHECK_VEL_CONTINUOUS_AT_START |
		Plan::NOT_CHECK_VEL_FOLLOWING_ERROR;



   
}
auto MovePressure::executeRT(PlanTarget &target)->int
{
    auto &param = std::any_cast<MovePressureParam&>(target.param);

	double RobotPosition[6];
	double RobotPositionJ[6];
	double RobotVelocity[6];
	double RobotAcceleration[6];
	double TorqueSensor[6];
	double X1[3];
	double X2[3];
	static double begin_pjs[6];
	static double step_pjs[6];
	static double stateTor0[6][3], stateTor1[6][3];
    static float FT0[6],FT_be[6];
    static double SumFtErr[6];
	// 访问主站 //
	auto controller = target.controller;

	// 获取当前起始点位置 //
	if (target.count == 1)
	{
		for (int i = 0; i < 6; ++i)
		{
			step_pjs[i] = target.model->motionPool()[i].mp();
			// controller->motionPool().at(i).setModeOfOperation(10);	//切换到电流控制
            SumFtErr[i]=0;
		}
	}


	if (!target.model->solverPool().at(1).kinPos())return -1;


	///* Using Jacobian, TransMatrix from ARIS
	double EndW[3], EndP[3], BaseV[3];
	double PqEnd[7], TransVector[16];
	target.model->generalMotionPool().at(0).getMpm(TransVector);
	target.model->generalMotionPool().at(0).getMpq(PqEnd);

    double dX[6] = { 0.00000, -0.0000, -0.0000, -0.0000, -0.0000, -0.0000 };
	double dTheta[6] = { 0 };

	float FT[6];
	uint16_t FTnum;
	auto conSensor = dynamic_cast<aris::control::EthercatController*>(target.controller);
	conSensor->ecSlavePool().at(6).readPdo(0x6030, 0x00, &FTnum, 16);
	conSensor->ecSlavePool().at(6).readPdo(0x6030, 0x01, &FT[0], 32);  //Fx
	conSensor->ecSlavePool().at(6).readPdo(0x6030, 0x02, &FT[1], 32);  //Fy
	conSensor->ecSlavePool().at(6).readPdo(0x6030, 0x03, &FT[2], 32);  //Fz
	conSensor->ecSlavePool().at(6).readPdo(0x6030, 0x04, &FT[3], 32);
	conSensor->ecSlavePool().at(6).readPdo(0x6030, 0x05, &FT[4], 32);
	conSensor->ecSlavePool().at(6).readPdo(0x6030, 0x06, &FT[5], 32);
    FT[0]=-FT[0];FT[3]=-FT[3];
	
	for (int i = 0;i < 6;i++)
	{
		RobotPositionJ[i] = target.model->motionPool()[i].mp();
		RobotPosition[i] = target.model->motionPool()[i].mp();
		RobotVelocity[i] = 0;
		RobotAcceleration[i] = 0;
		TorqueSensor[i] = FT[i];
	}



	// 获取当前起始点位置 //
	if (target.count == 1)
	{
		for (int j = 0; j < 6; j++)
		{
			stateTor0[j][0] = FT[j];
			FT0[j] = FT[j];
            FT_be[j]=FT[j];
		}
	}

    for (int j = 0; j < 6; j++)
    {
        if(abs(FT[j])<0.0001)
           FT[j]=FT_be[j];
    }
	

	for (int j = 0; j < 6; j++)
	{
        double A[3][3], B[3], CutFreq = 35;//SHANGHAI DIANQI EXP
        //CutFreq = 85;
		A[0][0] = 0; A[0][1] = 1; A[0][2] = 0;
		A[1][0] = 0; A[1][1] = 0; A[1][2] = 1;
		A[2][0] = -CutFreq * CutFreq * CutFreq;
		A[2][1] = -2 * CutFreq * CutFreq;
		A[2][2] = -2 * CutFreq;
		B[0] = 0; B[1] = 0;
		B[2] = -A[2][0];
		double intDT = 0.001;
		stateTor1[j][0] = stateTor0[j][0] + intDT * (A[0][0] * stateTor0[j][0] + A[0][1] * stateTor0[j][1] + A[0][2] * stateTor0[j][2] + B[0] * FT[j]);
		stateTor1[j][1] = stateTor0[j][1] + intDT * (A[1][0] * stateTor0[j][0] + A[1][1] * stateTor0[j][1] + A[1][2] * stateTor0[j][2] + B[1] * FT[j]);
		stateTor1[j][2] = stateTor0[j][2] + intDT * (A[2][0] * stateTor0[j][0] + A[2][1] * stateTor0[j][1] + A[2][2] * stateTor0[j][2] + B[2] * FT[j]);
	}


    double FT_KAI[6];
	for (int i = 0; i < 6; i++)
	{
        FT_KAI[i] = stateTor1[i][0] - FT0[i];//In KAI Coordinate
    }


    for (int i = 0; i < 3; i++)
    {
        if(FT_KAI[i]<1.0&&FT_KAI[i]>0)
            FT_KAI[i] = FT_KAI[i]*FT_KAI[i];//In KAI Coordinate
        else if(FT_KAI[i]<0&&FT_KAI[i]>-1.0)
            FT_KAI[i] = -FT_KAI[i]*FT_KAI[i];//In KAI Coordinate
    }

    for (int i = 3; i < 6; i++)
    {
        if(FT_KAI[i]<0.05&&FT_KAI[i]>0)
            FT_KAI[i] = 20*FT_KAI[i]*FT_KAI[i];//In KAI Coordinate
        else if(FT_KAI[i]<0&&FT_KAI[i]>-0.05)
            FT_KAI[i] = -20*FT_KAI[i]*FT_KAI[i];//In KAI Coordinate
    }


    double FT_YANG[6];
    FT_YANG[0]=FT_KAI[2];FT_YANG[1]=-FT_KAI[1];FT_YANG[2]=FT_KAI[0];
    FT_YANG[3]=FT_KAI[5];FT_YANG[4]=-FT_KAI[4];FT_YANG[5]=FT_KAI[3];

    double FmInWorld[6];

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

    //for(int i=0;i<6;i++)
        //SumFtErr[i]=SumFtErr[i]+
    if(abs(FmInWorld[2])>2)
       SumFtErr[2]=SumFtErr[2]+(FmInWorld[2]-(5))*DT;

    dX[2] = 1 * (FmInWorld[2]-(5)+0*SumFtErr[2]) / 820000;
    dX[3] = 1 * (FmInWorld[3]) / 4000;
    dX[4] = 1 * (FmInWorld[4]) / 4000;
    dX[5] = 1 * (FmInWorld[5]) / 4000;

    /*//  SHANGHAI DIANQI EXP
    dX[0] = 1 * (FmInWorld[0]) / 40000;
    dX[1] = 1 * (FmInWorld[1]) / 40000;
    dX[2] = 1 * (FmInWorld[2]) / 40000;
    dX[3] = 1 * (FmInWorld[3]) / 4000;
    dX[4] = 1 * (FmInWorld[4]) / 4000;
    dX[5] = 1 * (FmInWorld[5]) / 4000;
*/


   if(target.count >23000)
        dX[0]=-0.00001;
   if(target.count >45000)
        dX[0]=0.00001;
   if(target.count >67000)
        dX[0]=-0.00001;
   if(target.count >89000)
        dX[0]=0.00001;
   if(target.count >111000)
        dX[0]=-0.00001;
   if(target.count >133000)
        dX[0]=0.00001;

	if (target.count % 100 == 0)
	{

        cout << FmInWorld[2]<<"***"<<SumFtErr[2]<<"***"<<dX[4]<<"***"<<dX[5]<<"***"<<FT0[2]<<endl;

       //cout <<  FT_KAI[0]<<"***"<<FmInWorld[2]<<endl;

		cout << std::endl;

	}


	for (int j = 0; j < 6; j++)
	{
		if (dX[j] > 0.00025)
			dX[j] = 0.00025;
		if (dX[j] < -0.00025)
			dX[j] = -0.00025;
	}


	// 打印电流 //
	auto &cout = controller->mout();

	// log 电流 //
	auto &lout = controller->lout();

	// lout << target.model->motionPool()[0].mp() << ",";
	 //lout << target.model->motionPool()[1].mp() << ",";
	 //lout << target.model->motionPool()[2].mp() << ",";
	 //lout << target.model->motionPool()[3].mp() << ",";
	 //lout << target.model->motionPool()[4].mp() << ",";
	 //lout << target.model->motionPool()[5].mp() << ",";

    lout << FTnum << ",";
    //lout << FT[2] << ",";lout << dX[2] << ",";
    //lout << FmInWorld[2] << ",";lout << FT0[2] << ",";

    lout << FmInWorld[0] << ",";lout << FmInWorld[1] << ",";
    lout << FmInWorld[2] << ",";lout << FmInWorld[3] << ",";
    lout << FmInWorld[4] << ",";
   // lout << FT_KAI[0] << ",";lout << FT_KAI[1] << ",";
   // lout << FT_KAI[2] << ",";lout << FT_KAI[3] << ",";
    //lout << FT_KAI[4] << ",";lout << FT_KAI[5] << ",";




	//lout << stateTor1[2][0] << ",";lout << FT0[3] << ",";
   // lout << dX[0] << ",";

   // lout << dX[0] << ",";
   // lout << FT[1] << ",";lout << FT[2] << ",";
   // lout << FT[3] << ",";lout << FT[4] << ",";
   // lout << FT[5] << ",";lout << FT[6] << ",";
	lout << std::endl;



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


	auto &fwd = dynamic_cast<aris::dynamic::ForwardKinematicSolver&>(target.model->solverPool()[1]);
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
			U[7 * i] = U[7 * i] + 0.1;
		else
			U[7 * i] = U[7 * i] - 0.1;
	// 根据QR分解的结果求x，相当于Matlab中的 x = A\b //
	s_householder_utp_sov(6, 6, 1, rank, U, tau, p, dX, dTheta, 1e-10);

	// 根据QR分解的结果求广义逆，相当于Matlab中的 pinv(A) //
	double tau2[6];
	s_householder_utp2pinv(6, 6, rank, U, tau, p, pinv, tau2, 1e-10);

	// 根据QR分解的结果求广义逆，相当于Matlab中的 pinv(A)*b //
	s_mm(6, 1, 6, pinv, dX, dTheta);



	for (int i = 0; i < 6; i++)
	{
		if (dTheta[i] > 0.003)
			dTheta[i] = 0.003;
		if (dTheta[i] < -0.003)
			dTheta[i] = -0.003;
		//lout << dTheta[i] << ",";
	}


	//lout << std::endl;
	for (int i = 0; i < 6; i++)
	{
		dTheta[i] = dTheta[i] * DirectionFlag[i];

	}

	for (int i = 0; i < 6; i++)
	{
		step_pjs[i] = step_pjs[i] + dTheta[i];
        target.model->motionPool().at(i).setMp(step_pjs[i]);
	}



	for (int i = 0; i < 6; i++)
	{

		stateTor0[i][0] = stateTor1[i][0];
		stateTor0[i][1] = stateTor1[i][1];
		stateTor0[i][2] = stateTor1[i][2];
	}

    for (int j = 0; j < 6; j++)
    {
        FT_be[j]=FT[j];
    }
	return 150000000 - target.count;
   
}

MovePressure::MovePressure(const std::string &name) :Plan(name)
    {
	
        command().loadXmlStr(
			"<Command name=\"mvPre\">"
            "	<GroupParam>"
            "       <Param name=\"PressF\" default=\"0\"/>"
            "   </GroupParam>"
			"</Command>");
			
    }


struct MovePressureToolParam
{
	double PressF;


};


auto MovePressureTool::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
{
	MovePressureToolParam param;
	for (auto &p : params)
	{
		if (p.first == "PressF")
			param.PressF = std::stod(p.second);

	}

	target.param = param;

	target.option |=
		Plan::USE_TARGET_POS |
		//#ifdef WIN32
		Plan::NOT_CHECK_POS_MIN |
		Plan::NOT_CHECK_POS_MAX |
		Plan::NOT_CHECK_POS_CONTINUOUS |
		Plan::NOT_CHECK_POS_CONTINUOUS_AT_START |
		Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
		Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER_AT_START |
		Plan::NOT_CHECK_POS_FOLLOWING_ERROR |
		//#endif
		Plan::NOT_CHECK_VEL_MIN |
		Plan::NOT_CHECK_VEL_MAX |
		Plan::NOT_CHECK_VEL_CONTINUOUS |
		Plan::NOT_CHECK_VEL_CONTINUOUS_AT_START |
		Plan::NOT_CHECK_VEL_FOLLOWING_ERROR;




}
auto MovePressureTool::executeRT(PlanTarget &target)->int
{
	auto &param = std::any_cast<MovePressureToolParam&>(target.param);

	double RobotPosition[6];
	double RobotPositionJ[6];
	double RobotVelocity[6];
	double RobotAcceleration[6];
	double TorqueSensor[6];
	double X1[3];
	double X2[3];
	static double begin_pjs[6];
	static double step_pjs[6];
    static double stateTor0[6][3], stateTor1[6][3],EndP0[3];
    static double sT0[6][3], sT1[6][3];
	static float FT0[6], FT_be[6];

	// 访问主站 //
	auto controller = target.controller;

	// 获取当前起始点位置 //
	if (target.count == 1)
	{
		for (int i = 0; i < 6; ++i)
		{
			step_pjs[i] = target.model->motionPool()[i].mp();
			// controller->motionPool().at(i).setModeOfOperation(10);	//切换到电流控制
		}
	}


	if (!target.model->solverPool().at(1).kinPos())return -1;


	///* Using Jacobian, TransMatrix from ARIS
	double EndW[3], EndP[3], BaseV[3];
    double PqEnd[7], TransVector[16],NormalVector[3],CosNormalAng,SinNormalAng,NormalAng;
    double XBase[3]={1,0,0},YBase[3]={0,1,0},ZBase[3]={0,0,1};
    double CrossNormalZbase[3]={0};

	target.model->generalMotionPool().at(0).getMpm(TransVector);
	target.model->generalMotionPool().at(0).getMpq(PqEnd);
    NormalVector[0]=TransVector[0];NormalVector[1]=TransVector[4];NormalVector[2]=TransVector[8];

    crossVector(NormalVector,ZBase,CrossNormalZbase);
    CosNormalAng=NormalVector[2]/sqrt(NormalVector[0]*NormalVector[0]+NormalVector[1]*NormalVector[1]+NormalVector[2]*NormalVector[2]);
    SinNormalAng=sqrt(CrossNormalZbase[0]*CrossNormalZbase[0]+CrossNormalZbase[1]*CrossNormalZbase[1]+CrossNormalZbase[2]*CrossNormalZbase[2])/sqrt(NormalVector[0]*NormalVector[0]+NormalVector[1]*NormalVector[1]+NormalVector[2]*NormalVector[2]);
    NormalAng=atan2(SinNormalAng,CosNormalAng);

	double dX[6] = { 0.00000, -0.0000, -0.0000, -0.0000, -0.0000, -0.0000 };
	double dTheta[6] = { 0 };

	float FT[6];
	uint16_t FTnum;
	auto conSensor = dynamic_cast<aris::control::EthercatController*>(target.controller);
	conSensor->ecSlavePool().at(6).readPdo(0x6030, 0x00, &FTnum, 16);
	conSensor->ecSlavePool().at(6).readPdo(0x6030, 0x01, &FT[0], 32);  //Fx
	conSensor->ecSlavePool().at(6).readPdo(0x6030, 0x02, &FT[1], 32);  //Fy
	conSensor->ecSlavePool().at(6).readPdo(0x6030, 0x03, &FT[2], 32);  //Fz
	conSensor->ecSlavePool().at(6).readPdo(0x6030, 0x04, &FT[3], 32);
	conSensor->ecSlavePool().at(6).readPdo(0x6030, 0x05, &FT[4], 32);
	conSensor->ecSlavePool().at(6).readPdo(0x6030, 0x06, &FT[5], 32);
	FT[0] = -FT[0];FT[3] = -FT[3];

	for (int i = 0;i < 6;i++)
	{
		RobotPositionJ[i] = target.model->motionPool()[i].mp();
		RobotPosition[i] = target.model->motionPool()[i].mp();
		RobotVelocity[i] = 0;
		RobotAcceleration[i] = 0;
		TorqueSensor[i] = FT[i];
	}



	// 获取当前起始点位置 //
	if (target.count == 1)
	{
		for (int j = 0; j < 6; j++)
		{
			stateTor0[j][0] = FT[j];
			FT0[j] = FT[j];
			FT_be[j] = FT[j];
		}
        for (int i = 0;i < 3;i++)
            EndP0[i] = PqEnd[i];
	}

	for (int j = 0; j < 6; j++)
	{
		if (abs(FT[j]) < 0.0001)
			FT[j] = FT_be[j];
	}


	for (int j = 0; j < 6; j++)
	{
        double A[3][3], B[3], CutFreq = 35;//SHANGHAI DIANQI EXP
		//CutFreq = 85;
		A[0][0] = 0; A[0][1] = 1; A[0][2] = 0;
		A[1][0] = 0; A[1][1] = 0; A[1][2] = 1;
		A[2][0] = -CutFreq * CutFreq * CutFreq;
		A[2][1] = -2 * CutFreq * CutFreq;
		A[2][2] = -2 * CutFreq;
		B[0] = 0; B[1] = 0;
		B[2] = -A[2][0];
		double intDT = 0.001;
		stateTor1[j][0] = stateTor0[j][0] + intDT * (A[0][0] * stateTor0[j][0] + A[0][1] * stateTor0[j][1] + A[0][2] * stateTor0[j][2] + B[0] * FT[j]);
		stateTor1[j][1] = stateTor0[j][1] + intDT * (A[1][0] * stateTor0[j][0] + A[1][1] * stateTor0[j][1] + A[1][2] * stateTor0[j][2] + B[1] * FT[j]);
		stateTor1[j][2] = stateTor0[j][2] + intDT * (A[2][0] * stateTor0[j][0] + A[2][1] * stateTor0[j][1] + A[2][2] * stateTor0[j][2] + B[2] * FT[j]);
	}


	double FT_KAI[6];
	for (int i = 0; i < 6; i++)
	{
		FT_KAI[i] = stateTor1[i][0] - FT0[i];//In KAI Coordinate
	}


	for (int i = 0; i < 3; i++)
	{
		if (FT_KAI[i] < 1.0&&FT_KAI[i]>0)
			FT_KAI[i] = FT_KAI[i] * FT_KAI[i];//In KAI Coordinate
		else if (FT_KAI[i]<0 && FT_KAI[i]>-1.0)
			FT_KAI[i] = -FT_KAI[i] * FT_KAI[i];//In KAI Coordinate
	}

	for (int i = 3; i < 6; i++)
	{
		if (FT_KAI[i] < 0.05&&FT_KAI[i]>0)
			FT_KAI[i] = 20 * FT_KAI[i] * FT_KAI[i];//In KAI Coordinate
		else if (FT_KAI[i]<0 && FT_KAI[i]>-0.05)
			FT_KAI[i] = -20 * FT_KAI[i] * FT_KAI[i];//In KAI Coordinate
	}



	double dXpid[6] = { 0,0,0,0,0,0 };
    dXpid[2] = 1 * (FT_KAI[2] - (-5)) / 620000;
    dXpid[3] = 1 * (FT_KAI[3]) / 2000;
    dXpid[4] = 1 * (FT_KAI[4]) / 2000;
    dXpid[5] = 1 * (FT_KAI[5]) / 2000;

	double FT_YANG[6];
	FT_YANG[0] = dXpid[2];FT_YANG[1] = -dXpid[1];FT_YANG[2] = dXpid[0];
	FT_YANG[3] = dXpid[5];FT_YANG[4] = -dXpid[4];FT_YANG[5] = dXpid[3];


	double FmInWorld[6];

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

	for (int i = 0;i < 6;i++)
		dX[i] = FmInWorld[i];


	double TangentArc[3] = { 0 };
	static double TangentArc0[3] = { 0 };
	static double TangentArc1[3] = { 0 };
	static double TangentArc2[3] = { 0 };
	static bool MoveDirection = true;
	static bool MoveDirectionT = true, MoveDirectionF = false;
	static bool MoveDirectionChange = false;
	static int StartCount = 1000;
	double CosTheta1, CosTheta2;
	



	static double pArc, vArc, aArc,vArcMax=0.05;
	aris::Size t_count;

	double Square[4][3] = { {0,0.22,0.22},
							{0,0.22,0.42},
							{0,-0.22,0.42},
							{0,-0.22,0.22}};


	static double MoveLength = 0;
	static double DecLength = 0.01, LengthT = 0.2, LengthF = 0.05;//LengthT>LengthF

	LengthT = sqrt((Square[0][1] - Square[1][1])*(Square[0][1] - Square[1][1]) + (Square[0][2] - Square[1][2])*(Square[0][2] - Square[1][2]));
	double CountFmax = sqrt((Square[2][1] - Square[1][1])*(Square[2][1] - Square[1][1]) + (Square[2][2] - Square[1][2])*(Square[2][2] - Square[1][2])) / LengthF;


	double DecTime = 0, Dec = 0;
	static int count_offsetT=StartCount, count_offsetF = StartCount;
	static double vArcEndT = 0, vArcEndF = 0;
	static int CountT = 0, CountF = 0;

	double Ktemp, temp0, temp1;
	double CrossSurface[3] = { 0,1,0 };//YZ
	double ExtendSurface[3] = {0,0,-1 };
	temp0 = Square[1][1] - Square[0][1];temp1 = Square[1][2] - Square[0][2];
	ExtendSurface[1] = temp0 / sqrt(temp0*temp0 + temp1 * temp1); ExtendSurface[2] = temp1 / sqrt(temp0*temp0 + temp1 * temp1); ExtendSurface[0] = 0;


	temp0 = Square[2][1] - Square[1][1];temp1 = Square[2][2] - Square[1][2];
	CrossSurface[1] = temp0 / sqrt(temp0*temp0 + temp1 * temp1); CrossSurface[2] = temp1 / sqrt(temp0*temp0 + temp1 * temp1); CrossSurface[0] = 0;


	
	

	if (target.count > StartCount&&MoveDirectionT ==true&& MoveDirectionF ==false)
	{
		if (CountT % 2 == 0)
			MoveDirection = true;
		else
			MoveDirection = false;
		/*
		if (MoveDirection == true)
			if (LengthT < 0)
				LengthT = -LengthT;

		if (MoveDirection == false)
			if (LengthT > 0)
				LengthT = -LengthT;
         */
		if (abs(NormalVector[0]) < 0.01)
		{
			TangentArc1[0] = ExtendSurface[0]; TangentArc1[1] = ExtendSurface[1]; TangentArc1[2] = ExtendSurface[2];

			TangentArc2[0] = ExtendSurface[0]; TangentArc2[1] = -ExtendSurface[1]; TangentArc2[2] = -ExtendSurface[2];

			if (MoveDirection==true)
				for (int i = 0;i < 3;i++)
					TangentArc[i] = TangentArc1[i];

			if (MoveDirection == false)
			{
				for (int i = 0;i < 3;i++)
					TangentArc[i] = TangentArc2[i];
				
			}
			
		}
		else
		{
			temp0 = ExtendSurface[1] * ExtendSurface[1] + ExtendSurface[2] * ExtendSurface[2];
			temp1 = (ExtendSurface[1] * NormalVector[1] + ExtendSurface[2] * NormalVector[2]) / NormalVector[0];
			Ktemp = 1 / sqrt(temp0 + temp1 * temp1);

			TangentArc1[1] = Ktemp * ExtendSurface[1];
			TangentArc1[2] = Ktemp * ExtendSurface[2];
			TangentArc1[0] = -(NormalVector[1] * TangentArc0[1] + NormalVector[2] * TangentArc0[2]) / NormalVector[0];

			CosTheta1 = TangentArc1[0] * TangentArc0[0] + TangentArc1[1] * TangentArc0[1] + TangentArc1[2] * TangentArc0[2];

			temp0 = ExtendSurface[1] * ExtendSurface[1] + ExtendSurface[2] * ExtendSurface[2];
			temp1 = (ExtendSurface[1] * NormalVector[1] + ExtendSurface[2] * NormalVector[2]) / NormalVector[0];
			Ktemp = -1 / sqrt(temp0 + temp1 * temp1);

			TangentArc2[1] = Ktemp * ExtendSurface[1];
			TangentArc2[2] = Ktemp * ExtendSurface[2];
			TangentArc2[0] = -(NormalVector[1] * TangentArc0[1] + NormalVector[2] * TangentArc0[2]) / NormalVector[0];

			if (MoveDirection == true)
				for (int i = 0;i < 3;i++)
					TangentArc[i] = TangentArc1[i];

			if (MoveDirection == false)
				for (int i = 0;i < 3;i++)
					TangentArc[i] = TangentArc2[i];
		
		}


		if (MoveDirection)
			if (MoveLength < LengthT - DecLength)
			{
				aris::plan::moveAbsolute(target.count - count_offsetF + 1, 0, 1000, vArcMax / 1000, 0.05 / 1000 / 1000, 0.05 / 1000 / 1000, pArc, vArc, aArc, t_count);
				vArc = vArc * 1000;
				vArcEndT = vArc;
			}
			else
			{

				vArc = vArcEndT - 1*(DecLength-(LengthT-MoveLength)) / DecLength * vArcEndT;

				if (abs(vArc) < 0.0001)
				{
					MoveDirectionT = false;
					MoveDirectionF = true;
					count_offsetT = target.count;
					MoveDirectionChange = true;
					CountT = CountT + 1;
				}
			}

		if (!MoveDirection)
			if (MoveLength > (DecLength))
			{
				aris::plan::moveAbsolute(target.count - count_offsetF + 1, 0, 1000, vArcMax / 1000, 0.05 / 1000 / 1000, 0.05 / 1000 / 1000, pArc, vArc, aArc, t_count);
				vArc = vArc * 1000;
				vArcEndF = vArc;
			}
			else
			{
				vArc = vArcEndF - 1*(DecLength-MoveLength)/ DecLength * vArcEndF;

				if (abs(vArc) < 0.0001)
				{
					MoveDirectionT = false;
					MoveDirectionF = true;
					count_offsetT = target.count;
					MoveDirectionChange = true;
					CountT = CountT + 1;
				}
			}

	}


	if (target.count > StartCount&&MoveDirectionT == false && MoveDirectionF == true&& CountF< CountFmax)
	{
		if (CountF % 2 == 0)
		{
			MoveDirection = true;
		}
		else
		{
			MoveDirection = true;
		}
		if (abs(NormalVector[0]) < 0.01)
		{
			TangentArc1[0] = CrossSurface[0]; TangentArc1[1] = CrossSurface[1]; TangentArc1[2] = CrossSurface[2];
			CosTheta1 = TangentArc1[0] * TangentArc0[0] + TangentArc1[1] * TangentArc0[1] + TangentArc1[2] * TangentArc0[2];

				for (int i = 0;i < 3;i++)
					TangentArc[i] = TangentArc1[i];
		}
		else
		{
			temp0 = CrossSurface[1] * CrossSurface[1] + CrossSurface[2] * CrossSurface[2];
			temp1 = (CrossSurface[1] * NormalVector[1] + CrossSurface[2] * NormalVector[2]) / NormalVector[0];
			Ktemp = 1 / sqrt(temp0 + temp1 * temp1);

			TangentArc1[1] = Ktemp * CrossSurface[1];
			TangentArc1[2] = Ktemp * CrossSurface[2];
			TangentArc1[0] = -(NormalVector[1] * TangentArc0[1] + NormalVector[2] * TangentArc0[2]) / NormalVector[0];

				for (int i = 0;i < 3;i++)
					TangentArc[i] = TangentArc1[i];

		}


		if (MoveDirection)
		   {
				aris::plan::moveAbsolute(target.count - count_offsetT + 1, 0, LengthF, vArcMax / 1000, 0.05 / 1000 / 1000, 0.05 / 1000 / 1000, pArc, vArc, aArc, t_count);
				vArc = vArc * 1000;
				vArcEndT = vArc;
			}
		
			if (abs(vArc) < 0.0001&&aArc<0)
				{
					MoveDirectionT = true;
					MoveDirectionF = false;
					count_offsetF = target.count;
					MoveDirectionChange = true;
					CountF = CountF + 1;
				}
	

	}
	if(CountF > CountFmax-1)
	{
		vArc = 0;
	}


    if (target.count > StartCount)
    {
		if (MoveDirection)
		{
			dX[0] = 0*vArc * TangentArc[0]/1000;
			dX[1] = vArc * TangentArc[1] / 1000;
			dX[2] =  vArc * TangentArc[2] / 1000;
			
		}
		else
		{
			dX[0] = 0 * vArc * TangentArc[0] / 1000;
			dX[1] = vArc * TangentArc[1] / 1000;
			dX[2] = vArc * TangentArc[2] / 1000;
		}
		if (target.count > StartCount&&MoveDirectionT == true && MoveDirectionF == false)
			if(MoveDirection)
				MoveLength = MoveLength + sqrt(dX[1] * dX[1] + dX[2] * dX[2]);
			else
				MoveLength = MoveLength - sqrt(dX[1] * dX[1] + dX[2] * dX[2]);
    }





    //if(FT_KAI[2]<-12.5)
      //  ForceToMeng =9.38;
   // else
/*
    for (int j = 0; j < 6; j++)
    {
        double A[3][3], B[3], CutFreq = 10;//SHANGHAI DIANQI EXP
        //CutFreq = 85;
        A[0][0] = 0; A[0][1] = 1; A[0][2] = 0;
        A[1][0] = 0; A[1][1] = 0; A[1][2] = 1;
        A[2][0] = -CutFreq * CutFreq * CutFreq;
        A[2][1] = -2 * CutFreq * CutFreq;
        A[2][2] = -2 * CutFreq;
        B[0] = 0; B[1] = 0;
        B[2] = -A[2][0];
        double intDT = 0.001;

        if(target.count > start&&target.count < (start+1*interval-StopInt))
        {
        if(FT_KAI[j]<-14)
            FT_KAI[j]=-10.59+sin(target.count);
        if(FT_KAI[j]>-7)
            FT_KAI[j]=-9.37+sin(target.count);
        }
        sT1[j][0] = sT0[j][0] + intDT * (A[0][0] * sT0[j][0] + A[0][1] * sT0[j][1] + A[0][2] * sT0[j][2] + B[0] * FT_KAI[j]);
        sT1[j][1] = sT0[j][1] + intDT * (A[1][0] * sT0[j][0] + A[1][1] * sT0[j][1] + A[1][2] * sT0[j][2] + B[1] * FT_KAI[j]);
        sT1[j][2] = sT0[j][2] + intDT * (A[2][0] * sT0[j][0] + A[2][1] * sT0[j][1] + A[2][2] * sT0[j][2] + B[2] * FT_KAI[j]);
    }*/

    ForceToMeng = sT1[2][0];
    if(ForceToMeng<-14)
        ForceToMeng=-11.59;
    if(ForceToMeng>-8)
        ForceToMeng=-9.37;

	ForceToMeng = vArc;
	TimeToMeng = target.count/1000.0;
	if (target.count % 300 == 0)
	{

        cout << FT_KAI[2] << "*" << vArc << "*" << MoveDirection <<"*"<< MoveLength<< endl;

//cout << FT_KAI[2] << "*" << NormalAng << "*" << TransVector[0] << "*" << TransVector[1] << "*" << TransVector[2] << "*" << FT0[2] << endl;
//cout << FT_KAI[2] << "*" << NormalAng << "*" << TransVector[4] << "*" << TransVector[5] << "*" << TransVector[6] << "*" << FT0[2] << endl;
//cout << FT_KAI[2] << "*" << NormalAng << "*" << TransVector[8] << "*" << TransVector[9] << "*" << TransVector[10] << "*" << FT0[2] << endl;

		//cout <<  FT_KAI[0]<<"***"<<FmInWorld[2]<<endl;

		cout << std::endl;

	}


	for (int j = 0; j < 6; j++)
	{
		if (dX[j] > 0.00025)
			dX[j] = 0.00025;
		if (dX[j] < -0.00025)
			dX[j] = -0.00025;
	}


	// 打印电流 //
	auto &cout = controller->mout();

	// log 电流 //
	auto &lout = controller->lout();

	// lout << target.model->motionPool()[0].mp() << ",";
	 //lout << target.model->motionPool()[1].mp() << ",";
	 //lout << target.model->motionPool()[2].mp() << ",";
	 //lout << target.model->motionPool()[3].mp() << ",";
	 //lout << target.model->motionPool()[4].mp() << ",";
	 //lout << target.model->motionPool()[5].mp() << ",";
	lout << vArc << endl;
	//lout << FTnum << ",";
	//lout << FT[2] << ",";lout << dX[2] << ",";
	//lout << FmInWorld[2] << ",";lout << FT0[2] << ",";

    //lout << stateTor1[0][0] << ",";lout << stateTor1[1][0] << ",";
    //lout << stateTor1[2][0] << ",";lout << stateTor1[3][0] << ",";
   // lout << stateTor1[4][0] << ",";lout << stateTor1[5][0] << ",";
   // lout << FT_KAI[0] << ",";lout << FT_KAI[1] << ",";
    //lout << FT_KAI[2] << ",";lout << FT_KAI[3] << ",";
    //lout << FT_KAI[4] << ",";lout << FT_KAI[5] << ",";


     //lout << FT[0] << ",";lout << FT[1] << ",";
     //lout << FT[2] << ",";lout << FT[3] << ",";
     //lout << FT[4] << ",";lout << FT[5] << ",";
	//lout << std::endl;



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


	auto &fwd = dynamic_cast<aris::dynamic::ForwardKinematicSolver&>(target.model->solverPool()[1]);
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
			U[7 * i] = U[7 * i] + 0.1;
		else
			U[7 * i] = U[7 * i] - 0.1;
	// 根据QR分解的结果求x，相当于Matlab中的 x = A\b //
	s_householder_utp_sov(6, 6, 1, rank, U, tau, p, dX, dTheta, 1e-10);

	// 根据QR分解的结果求广义逆，相当于Matlab中的 pinv(A) //
	double tau2[6];
	s_householder_utp2pinv(6, 6, rank, U, tau, p, pinv, tau2, 1e-10);

	// 根据QR分解的结果求广义逆，相当于Matlab中的 pinv(A)*b //
	s_mm(6, 1, 6, pinv, dX, dTheta);



	for (int i = 0; i < 6; i++)
	{
		if (dTheta[i] > 0.003)
			dTheta[i] = 0.003;
		if (dTheta[i] < -0.003)
			dTheta[i] = -0.003;
		//lout << dTheta[i] << ",";
	}


	//lout << std::endl;
	for (int i = 0; i < 6; i++)
	{
		dTheta[i] = dTheta[i] * DirectionFlag[i];

	}

	for (int i = 0; i < 6; i++)
	{
		step_pjs[i] = step_pjs[i] + dTheta[i];
        target.model->motionPool().at(i).setMp(step_pjs[i]);
	}



	for (int i = 0; i < 6; i++)
	{

		stateTor0[i][0] = stateTor1[i][0];
		stateTor0[i][1] = stateTor1[i][1];
		stateTor0[i][2] = stateTor1[i][2];

        sT0[i][0] = sT1[i][0];
        sT0[i][1] = sT1[i][1];
        sT0[i][2] = sT1[i][2];

	}

	for (int j = 0; j < 6; j++)
	{
		FT_be[j] = FT[j];
	}
	for (int j = 0; j < 3; j++)
	{
		TangentArc0[j] = TangentArc[j];
	}

	return 150000000 - target.count;

}

MovePressureTool::MovePressureTool(const std::string &name) :Plan(name)
{

	command().loadXmlStr(
		"<Command name=\"mvPreT\">"
		"	<GroupParam>"
		"       <Param name=\"PressF\" default=\"0\"/>"
		"   </GroupParam>"
		"</Command>");

}


auto GetForce::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
{

	double FT[2] = { 0,0 };

	std::any cur_a = double(0);
	target.server->getRtData([&](aris::server::ControlServer& cs, std::any &data)->void
	{
		FT[0] = TimeToMeng;
		FT[1] = ForceToMeng;
		//std::any_cast<double&>(data) = cs.controller().motionPool().at(i).actualCur();
	}, cur_a);

	
	std::string ret(reinterpret_cast<char*>(&FT),2 * sizeof(double));
	target.ret = ret;
	target.option |= NOT_RUN_EXECUTE_FUNCTION | NOT_PRINT_CMD_INFO | NOT_PRINT_CMD_INFO;
}
auto GetForce::collectNrt(PlanTarget &target)->void {}
GetForce::GetForce(const std::string &name) : Plan(name)
{
	command().loadXmlStr(
		"<Command name=\"GetForce\">"
		"</Command>");
}









struct MoveFeedParam
{
	double PressF;


};

auto MoveFeed::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
{
	MoveFeedParam param;
	for (auto &p : params)
	{
		if (p.first == "PressF")
			param.PressF = std::stod(p.second);

	}

	target.param = param;

	target.option |=
		Plan::USE_TARGET_POS |
		//#ifdef WIN32
		Plan::NOT_CHECK_POS_MIN |
		Plan::NOT_CHECK_POS_MAX |
		Plan::NOT_CHECK_POS_CONTINUOUS |
		Plan::NOT_CHECK_POS_CONTINUOUS_AT_START |
		Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
		Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER_AT_START |
		Plan::NOT_CHECK_POS_FOLLOWING_ERROR |
		//#endif
		Plan::NOT_CHECK_VEL_MIN |
		Plan::NOT_CHECK_VEL_MAX |
		Plan::NOT_CHECK_VEL_CONTINUOUS |
		Plan::NOT_CHECK_VEL_CONTINUOUS_AT_START |
		Plan::NOT_CHECK_VEL_FOLLOWING_ERROR;




}
auto MoveFeed::executeRT(PlanTarget &target)->int
{
	auto &param = std::any_cast<MoveFeedParam&>(target.param);

	double RobotPosition[6];
	double RobotPositionJ[6];
	double RobotVelocity[6];
	double RobotAcceleration[6];
	double TorqueSensor[6];
	double X1[3];
	double X2[3];
	static double begin_pjs[6];
	static double step_pjs[6];
	static double stateTor0[6][3], stateTor1[6][3];
	static float FT0[6], FT_be[6];
	static double SumFtErr[6];
	// 访问主站 //
	auto controller = target.controller;

	// 获取当前起始点位置 //
	if (target.count == 1)
	{
		for (int i = 0; i < 6; ++i)
		{
			step_pjs[i] = target.model->motionPool()[i].mp();
			// controller->motionPool().at(i).setModeOfOperation(10);	//切换到电流控制
			SumFtErr[i] = 0;
		}
	}


	if (!target.model->solverPool().at(1).kinPos())return -1;


	///* Using Jacobian, TransMatrix from ARIS
	double EndW[3], EndP[3], BaseV[3];
	double PqEnd[7], TransVector[16];
	target.model->generalMotionPool().at(0).getMpm(TransVector);
	target.model->generalMotionPool().at(0).getMpq(PqEnd);

	double dX[6] = { 0.00000, -0.0000, -0.0000, -0.0000, -0.0000, -0.0000 };
	double dTheta[6] = { 0 };

	float FT[6];
	uint16_t FTnum;
	auto conSensor = dynamic_cast<aris::control::EthercatController*>(target.controller);
	conSensor->ecSlavePool().at(6).readPdo(0x6030, 0x00, &FTnum, 16);
	conSensor->ecSlavePool().at(6).readPdo(0x6030, 0x01, &FT[0], 32);  //Fx
	conSensor->ecSlavePool().at(6).readPdo(0x6030, 0x02, &FT[1], 32);  //Fy
	conSensor->ecSlavePool().at(6).readPdo(0x6030, 0x03, &FT[2], 32);  //Fz
	conSensor->ecSlavePool().at(6).readPdo(0x6030, 0x04, &FT[3], 32);
	conSensor->ecSlavePool().at(6).readPdo(0x6030, 0x05, &FT[4], 32);
	conSensor->ecSlavePool().at(6).readPdo(0x6030, 0x06, &FT[5], 32);
	FT[0] = -FT[0];FT[3] = -FT[3];

	for (int i = 0;i < 6;i++)
	{
		RobotPositionJ[i] = target.model->motionPool()[i].mp();
		RobotPosition[i] = target.model->motionPool()[i].mp();
		RobotVelocity[i] = 0;
		RobotAcceleration[i] = 0;
		TorqueSensor[i] = FT[i];
	}



	// 获取当前起始点位置 //
	if (target.count == 1)
	{
		for (int j = 0; j < 6; j++)
		{
			stateTor0[j][0] = FT[j];
			FT0[j] = FT[j];
			FT_be[j] = FT[j];
		}
	}

	for (int j = 0; j < 6; j++)
	{
		if (abs(FT[j]) < 0.0001)
			FT[j] = FT_be[j];
	}


	for (int j = 0; j < 6; j++)
	{
		double A[3][3], B[3], CutFreq = 35;//SHANGHAI DIANQI EXP
		//CutFreq = 85;
		A[0][0] = 0; A[0][1] = 1; A[0][2] = 0;
		A[1][0] = 0; A[1][1] = 0; A[1][2] = 1;
		A[2][0] = -CutFreq * CutFreq * CutFreq;
		A[2][1] = -2 * CutFreq * CutFreq;
		A[2][2] = -2 * CutFreq;
		B[0] = 0; B[1] = 0;
		B[2] = -A[2][0];
		double intDT = 0.001;
		stateTor1[j][0] = stateTor0[j][0] + intDT * (A[0][0] * stateTor0[j][0] + A[0][1] * stateTor0[j][1] + A[0][2] * stateTor0[j][2] + B[0] * FT[j]);
		stateTor1[j][1] = stateTor0[j][1] + intDT * (A[1][0] * stateTor0[j][0] + A[1][1] * stateTor0[j][1] + A[1][2] * stateTor0[j][2] + B[1] * FT[j]);
		stateTor1[j][2] = stateTor0[j][2] + intDT * (A[2][0] * stateTor0[j][0] + A[2][1] * stateTor0[j][1] + A[2][2] * stateTor0[j][2] + B[2] * FT[j]);
	}


	double FT_KAI[6];
	for (int i = 0; i < 6; i++)
	{
		FT_KAI[i] = stateTor1[i][0] - FT0[i];//In KAI Coordinate
	}


	for (int i = 0; i < 3; i++)
	{
		if (FT_KAI[i] < 1.0&&FT_KAI[i]>0)
			FT_KAI[i] = FT_KAI[i] * FT_KAI[i];//In KAI Coordinate
		else if (FT_KAI[i]<0 && FT_KAI[i]>-1.0)
			FT_KAI[i] = -FT_KAI[i] * FT_KAI[i];//In KAI Coordinate
	}

	for (int i = 3; i < 6; i++)
	{
		if (FT_KAI[i] < 0.05&&FT_KAI[i]>0)
			FT_KAI[i] = 20 * FT_KAI[i] * FT_KAI[i];//In KAI Coordinate
		else if (FT_KAI[i]<0 && FT_KAI[i]>-0.05)
			FT_KAI[i] = -20 * FT_KAI[i] * FT_KAI[i];//In KAI Coordinate
	}


	double FT_YANG[6];
	FT_YANG[0] = FT_KAI[2];FT_YANG[1] = -FT_KAI[1];FT_YANG[2] = FT_KAI[0];
	FT_YANG[3] = FT_KAI[5];FT_YANG[4] = -FT_KAI[4];FT_YANG[5] = FT_KAI[3];

	double FmInWorld[6];

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


	//if (abs(FmInWorld[2]) > 2)
	//	SumFtErr[2] = SumFtErr[2] + (FmInWorld[2] - (5))*DT;
	double ContactForceXY = 0,ThetaXY=0,dXY=0;
    static bool FreeFlag=true;
	ContactForceXY = sqrt(FmInWorld[0] * FmInWorld[0] + FmInWorld[1] * FmInWorld[1]);

    if(ContactForceXY>1||FreeFlag==false)
    {
        if(abs(FmInWorld[0])>0.01)
              ThetaXY = atan(FmInWorld[1] / FmInWorld[0]);
        else
               ThetaXY =1.57;
    dXY = 1 * (FmInWorld[1] - (-5)) / 820000;

    dX[0] = 0.00001;
    dX[1] = dXY;
	dX[2] = 0 * (FmInWorld[2] - (5) + 0 * SumFtErr[2]) / 820000;
	dX[3] = 0 * (FmInWorld[3]) / 4000;
	dX[4] = 0 * (FmInWorld[4]) / 4000;
	dX[5] = 0 * (FmInWorld[5]) / 4000;
    FreeFlag=false;
    }

    if(ContactForceXY<1&&FreeFlag)
    {
        dX[0] = 0.00001;
        dX[1] = 0.00001;
    }


/*
	if (target.count > 23000)
		dX[0] = -0.00001;
	if (target.count > 45000)
		dX[0] = 0.00001;
	if (target.count > 67000)
		dX[0] = -0.00001;
	if (target.count > 89000)
		dX[0] = 0.00001;
	if (target.count > 111000)
		dX[0] = -0.00001;
	if (target.count > 133000)
		dX[0] = 0.00001;
*/
    if (target.count % 100 == 0)
	{
        cout <<"******"<<endl;
        cout << FmInWorld[1] << "***" << ThetaXY << "***" << dX[0] << "***" << dX[1] << "***" << FT0[2] << endl;

		//cout <<  FT_KAI[0]<<"***"<<FmInWorld[2]<<endl;

		cout << std::endl;

	}


	for (int j = 0; j < 6; j++)
	{
		if (dX[j] > 0.00025)
			dX[j] = 0.00025;
		if (dX[j] < -0.00025)
			dX[j] = -0.00025;
	}


	// 打印电流 //
	auto &cout = controller->mout();

	// log 电流 //
	auto &lout = controller->lout();

	// lout << target.model->motionPool()[0].mp() << ",";
	 //lout << target.model->motionPool()[1].mp() << ",";
	 //lout << target.model->motionPool()[2].mp() << ",";
	 //lout << target.model->motionPool()[3].mp() << ",";
	 //lout << target.model->motionPool()[4].mp() << ",";
	 //lout << target.model->motionPool()[5].mp() << ",";

	lout << FTnum << ",";
	//lout << FT[2] << ",";lout << dX[2] << ",";
	//lout << FmInWorld[2] << ",";lout << FT0[2] << ",";

	lout << FmInWorld[0] << ",";lout << FmInWorld[1] << ",";
	lout << FmInWorld[2] << ",";lout << FmInWorld[3] << ",";
	lout << FmInWorld[4] << ",";
	// lout << FT_KAI[0] << ",";lout << FT_KAI[1] << ",";
	// lout << FT_KAI[2] << ",";lout << FT_KAI[3] << ",";
	 //lout << FT_KAI[4] << ",";lout << FT_KAI[5] << ",";




	 //lout << stateTor1[2][0] << ",";lout << FT0[3] << ",";
	// lout << dX[0] << ",";

	// lout << dX[0] << ",";
	// lout << FT[1] << ",";lout << FT[2] << ",";
	// lout << FT[3] << ",";lout << FT[4] << ",";
	// lout << FT[5] << ",";lout << FT[6] << ",";
	lout << std::endl;



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


	auto &fwd = dynamic_cast<aris::dynamic::ForwardKinematicSolver&>(target.model->solverPool()[1]);
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
			U[7 * i] = U[7 * i] + 0.1;
		else
			U[7 * i] = U[7 * i] - 0.1;
	// 根据QR分解的结果求x，相当于Matlab中的 x = A\b //
	s_householder_utp_sov(6, 6, 1, rank, U, tau, p, dX, dTheta, 1e-10);

	// 根据QR分解的结果求广义逆，相当于Matlab中的 pinv(A) //
	double tau2[6];
	s_householder_utp2pinv(6, 6, rank, U, tau, p, pinv, tau2, 1e-10);

	// 根据QR分解的结果求广义逆，相当于Matlab中的 pinv(A)*b //
	s_mm(6, 1, 6, pinv, dX, dTheta);



	for (int i = 0; i < 6; i++)
	{
		if (dTheta[i] > 0.003)
			dTheta[i] = 0.003;
		if (dTheta[i] < -0.003)
			dTheta[i] = -0.003;
		//lout << dTheta[i] << ",";
	}


	//lout << std::endl;
	for (int i = 0; i < 6; i++)
	{
		dTheta[i] = dTheta[i] * DirectionFlag[i];

	}

	for (int i = 0; i < 6; i++)
	{
		step_pjs[i] = step_pjs[i] + dTheta[i];
        target.model->motionPool().at(i).setMp(step_pjs[i]);
	}



	for (int i = 0; i < 6; i++)
	{

		stateTor0[i][0] = stateTor1[i][0];
		stateTor0[i][1] = stateTor1[i][1];
		stateTor0[i][2] = stateTor1[i][2];
	}

	for (int j = 0; j < 6; j++)
	{
		FT_be[j] = FT[j];
	}
	return 150000000 - target.count;

}

MoveFeed::MoveFeed(const std::string &name) :Plan(name)
{

	command().loadXmlStr(
		"<Command name=\"mvFe\">"
		"	<GroupParam>"
		"       <Param name=\"PressF\" default=\"0\"/>"
		"   </GroupParam>"
		"</Command>");

}









struct MoveJointParam
{
	double PressF;

};

auto MoveJoint::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
{
	MoveJointParam param;
	for (auto &p : params)
	{
		if (p.first == "PressF")
			param.PressF = std::stod(p.second);

	}

	target.param = param;

	target.option |=
		Plan::USE_TARGET_POS |
		//#ifdef WIN32
		Plan::NOT_CHECK_POS_MIN |
		Plan::NOT_CHECK_POS_MAX |
		Plan::NOT_CHECK_POS_CONTINUOUS |
		Plan::NOT_CHECK_POS_CONTINUOUS_AT_START |
		Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
		Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER_AT_START |
		Plan::NOT_CHECK_POS_FOLLOWING_ERROR |
		//#endif
		Plan::NOT_CHECK_VEL_MIN |
		Plan::NOT_CHECK_VEL_MAX |
		Plan::NOT_CHECK_VEL_CONTINUOUS |
		Plan::NOT_CHECK_VEL_CONTINUOUS_AT_START |
		Plan::NOT_CHECK_VEL_FOLLOWING_ERROR;




}
auto MoveJoint::executeRT(PlanTarget &target)->int
{
	auto &param = std::any_cast<MoveJointParam&>(target.param);

	double RobotPosition[6];
	double RobotPositionJ[6];
	double RobotVelocity[6];
	double RobotAcceleration[6];
	double TorqueSensor[6];
	double X1[3];
	double X2[3];
	static double begin_pjs[6];
	static double step_pjs[6];
	static double stateTor0[6][3], stateTor1[6][3];
	static float FT0[6], FT_be[6];

	// 访问主站 //
	auto controller = target.controller;

	// 获取当前起始点位置 //
	if (target.count == 1)
	{
		for (int i = 0; i < 6; ++i)
		{
			step_pjs[i] = target.model->motionPool()[i].mp();
			// controller->motionPool().at(i).setModeOfOperation(10);	//切换到电流控制
		}
	}


	if (!target.model->solverPool().at(1).kinPos())return -1;


	///* Using Jacobian, TransMatrix from ARIS
	double EndW[3], EndP[3], BaseV[3];
	double PqEnd[7], TransVector[16];
	target.model->generalMotionPool().at(0).getMpm(TransVector);
	target.model->generalMotionPool().at(0).getMpq(PqEnd);

	double dX[6] = { 0.00000, -0.0000, -0.0000, -0.0000, -0.0000, -0.0000 };
	double dTheta[6] = { 0 };

	float FT[6];
	uint16_t FTnum;
	auto conSensor = dynamic_cast<aris::control::EthercatController*>(target.controller);
	conSensor->ecSlavePool().at(6).readPdo(0x6030, 0x00, &FTnum, 16);
	conSensor->ecSlavePool().at(6).readPdo(0x6030, 0x01, &FT[0], 32);  //Fx
	conSensor->ecSlavePool().at(6).readPdo(0x6030, 0x02, &FT[1], 32);  //Fy
	conSensor->ecSlavePool().at(6).readPdo(0x6030, 0x03, &FT[2], 32);  //Fz
	conSensor->ecSlavePool().at(6).readPdo(0x6030, 0x04, &FT[3], 32);
	conSensor->ecSlavePool().at(6).readPdo(0x6030, 0x05, &FT[4], 32);
	conSensor->ecSlavePool().at(6).readPdo(0x6030, 0x06, &FT[5], 32);
	FT[0] = -FT[0];FT[3] = -FT[3];

	for (int i = 0;i < 6;i++)
	{
		RobotPositionJ[i] = target.model->motionPool()[i].mp();
		RobotPosition[i] = target.model->motionPool()[i].mp();
		RobotVelocity[i] = 0;
		RobotAcceleration[i] = 0;
		TorqueSensor[i] = FT[i];
	}

	// 获取当前起始点位置 //
	if (target.count == 1)
	{
		for (int j = 0; j < 6; j++)
		{
			stateTor0[j][0] = FT[j];
			FT0[j] = FT[j];
			FT_be[j] = FT[j];
		}
	}

	for (int j = 0; j < 6; j++)
	{
		if (abs(FT[j]) < 0.0001)
			FT[j] = FT_be[j];
	}

	for (int j = 0; j < 6; j++)
	{
        double A[3][3], B[3], CutFreq = 15;//SHANGHAI DIANQI EXP
		
		A[0][0] = 0; A[0][1] = 1; A[0][2] = 0;
		A[1][0] = 0; A[1][1] = 0; A[1][2] = 1;
		A[2][0] = -CutFreq * CutFreq * CutFreq;
		A[2][1] = -2 * CutFreq * CutFreq;
		A[2][2] = -2 * CutFreq;
		B[0] = 0; B[1] = 0;
		B[2] = -A[2][0];
		double intDT = 0.001;
		stateTor1[j][0] = stateTor0[j][0] + intDT * (A[0][0] * stateTor0[j][0] + A[0][1] * stateTor0[j][1] + A[0][2] * stateTor0[j][2] + B[0] * FT[j]);
		stateTor1[j][1] = stateTor0[j][1] + intDT * (A[1][0] * stateTor0[j][0] + A[1][1] * stateTor0[j][1] + A[1][2] * stateTor0[j][2] + B[1] * FT[j]);
		stateTor1[j][2] = stateTor0[j][2] + intDT * (A[2][0] * stateTor0[j][0] + A[2][1] * stateTor0[j][1] + A[2][2] * stateTor0[j][2] + B[2] * FT[j]);
	}

	double FT_KAI[6];
	for (int i = 0; i < 6; i++)
	{
		FT_KAI[i] = stateTor1[i][0] - FT0[i];//In KAI Coordinate
	}

	double FT_YANG[6];
	FT_YANG[0] = FT_KAI[2];FT_YANG[1] = -FT_KAI[1];FT_YANG[2] = FT_KAI[0];
	FT_YANG[3] = FT_KAI[5];FT_YANG[4] = -FT_KAI[4];FT_YANG[5] = FT_KAI[3];

	double FmInWorld[6];

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


	if (target.count % 100 == 0)
	{

        //cout << FmInWorld[0] << "***" << FmInWorld[1] << "***" << FmInWorld[2] << "***" << FmInWorld[3] << "***" <<FmInWorld[4] << "***" <<FmInWorld[5] << "***" << FT0[2] << endl;

		//cout <<  FT_KAI[0]<<"***"<<FmInWorld[2]<<endl;

		cout << std::endl;

	}


	for (int j = 0; j < 6; j++)
	{
		if (dX[j] > 0.00025)
			dX[j] = 0.00025;
		if (dX[j] < -0.00025)
			dX[j] = -0.00025;
	}


	// 打印电流 //
	auto &cout = controller->mout();

	// log 电流 //
	auto &lout = controller->lout();

	lout << FTnum << ",";
	//lout << FT[2] << ",";lout << dX[2] << ",";
	//lout << FmInWorld[2] << ",";lout << FT0[2] << ",";

	lout << FmInWorld[0] << ",";lout << FmInWorld[1] << ",";
	lout << FmInWorld[2] << ",";lout << FmInWorld[3] << ",";
	lout << FmInWorld[4] << ",";
	// lout << FT_KAI[0] << ",";lout << FT_KAI[1] << ",";
	// lout << FT_KAI[2] << ",";lout << FT_KAI[3] << ",";
	 //lout << FT_KAI[4] << ",";lout << FT_KAI[5] << ",";

	// lout << dX[0] << ",";
	// lout << FT[1] << ",";lout << FT[2] << ",";
	// lout << FT[3] << ",";lout << FT[4] << ",";
	// lout << FT[5] << ",";lout << FT[6] << ",";
	lout << std::endl;



	///* Using Jacobian, TransMatrix from ARIS
    auto &fwd = dynamic_cast<aris::dynamic::ForwardKinematicSolver&>(target.model->solverPool()[1]);
	fwd.cptJacobi();
	double pinv[36];
	double tempS[6][6] = { 0 };
	for (int i = 0;i < 6;i++)
		for (int j = 0;j < 6;j++)
			if (i == j)
				tempS[i][j] = 1;
	tempS[0][4] = -PqEnd[2];tempS[0][5] = PqEnd[1];
	tempS[1][3] = PqEnd[2];tempS[1][5] = -PqEnd[0];
	tempS[2][3] = -PqEnd[1];tempS[2][4] = PqEnd[0];
	double tempVec[36] = { 0 };
	for (int i = 0;i < 6;i++)
		for (int j = 0;j < 6;j++)
			tempVec[6 * i + j] = tempS[i][j];

	double U[36], tau[6];
	aris::Size p[6];
	aris::Size rank;

	s_householder_utp(6, 6, tempVec, U, tau, p, rank, 1e-10);
	// 根据QR分解的结果求广义逆，相当于Matlab中的 pinv(A) //
	double tau2[6];
	s_householder_utp2pinv(6, 6, rank, U, tau, p, pinv, tau2, 1e-10);

	// 根据QR分解的结果求广义逆，相当于Matlab中的 pinv(A)*b //
	double JacobEnd[36] = { 0 };
	s_mm(6, 6, 6, pinv, fwd.Jf(), JacobEnd);
	//robotDemo.jointIncrement(RobotPositionJ, dX, dTheta);
    double JacobEndTrans[36] = { 0 };
    double temp[6][6] = { 0 };
    double temp1[6][6] = { 0 };
    for (int i = 0;i < 6;i++)
        for (int j = 0;j < 6;j++)
            temp[i][j] = JacobEnd[i*6+j];
    for (int i = 0;i < 6;i++)
        for (int j = 0;j < 6;j++)
            temp1[i][j] = temp[j][i];
    for (int i = 0;i < 6;i++)
        for (int j = 0;j < 6;j++)
            JacobEndTrans[6 * i + j] = temp1[i][j];



    FmInWorld[0]=-2; FmInWorld[1]=0; FmInWorld[2]=0; FmInWorld[3]=0; FmInWorld[4]=0; FmInWorld[5]=0;
	double JoinTau[6] = { 0 };
    s_mm(6, 1, 6, JacobEndTrans, FmInWorld, JoinTau);
    if (target.count % 100 == 0)
    {

        cout << JoinTau[0] << "***" << JoinTau[1] << "***" << JoinTau[2] << "***" << JoinTau[3] << "***" <<JoinTau[4] << "***" <<JoinTau[5] << "***" << FT0[2] << endl;

        //cout <<  FT_KAI[0]<<"***"<<FmInWorld[2]<<endl;

        cout << std::endl;

    }
    //for (int i = 0;i < 6;i++)
       // dTheta[i] = JoinTau[i] / 10000;

     dTheta[0] = JoinTau[0] / 10000;
     dTheta[1] = JoinTau[1] / 8000;
     dTheta[2] = JoinTau[2] / 8000;
     dTheta[3] = JoinTau[3] / 2000;
     dTheta[4] = JoinTau[4] / 2000;
     dTheta[5] = JoinTau[5] / 2000;
	for (int i = 0; i < 6; i++)
	{
		if (dTheta[i] > 0.003)
			dTheta[i] = 0.003;
		if (dTheta[i] < -0.003)
			dTheta[i] = -0.003;
		//lout << dTheta[i] << ",";
	}

	for (int i = 0; i < 6; i++)
	{
		dTheta[i] = dTheta[i] * DirectionFlag[i];

	}

	for (int i = 0; i < 6; i++)
	{
		step_pjs[i] = step_pjs[i] + dTheta[i];
        target.model->motionPool().at(i).setMp(step_pjs[i]);
	}

	for (int i = 0; i < 6; i++)
	{

		stateTor0[i][0] = stateTor1[i][0];
		stateTor0[i][1] = stateTor1[i][1];
		stateTor0[i][2] = stateTor1[i][2];
	}

	for (int j = 0; j < 6; j++)
	{
		FT_be[j] = FT[j];
	}
	return 150000000 - target.count;

}

MoveJoint::MoveJoint(const std::string &name) :Plan(name)
{

	command().loadXmlStr(
		"<Command name=\"mvJoint\">"
		"	<GroupParam>"
		"       <Param name=\"PressF\" default=\"0\"/>"
		"   </GroupParam>"
		"</Command>");

}

