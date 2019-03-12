#include "sixdistalfc.h"
#include <math.h>
#include"robotconfig.h"
#include"sixdistaldynamics.h"
using namespace std;
using namespace aris::plan;
using namespace aris::dynamic;
using namespace CONFIG;
using namespace sixDistalDynamicsInt;
/// \brief





robotconfig robotDemo;
sixdistaldynamics sixDistalMatrix;
std::array<double, 6> estParas;
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
		Plan::NOT_CHECK_VEL_MIN |
		Plan::NOT_CHECK_VEL_MAX |
		Plan::NOT_CHECK_VEL_CONTINUOUS |
		Plan::NOT_CHECK_VEL_CONTINUOUS_AT_START |
		Plan::NOT_CHECK_VEL_FOLLOWING_ERROR;
	
   
}
auto MoveXYZ::executeRT(PlanTarget &target)->int
{
			auto &param = std::any_cast<MoveXYZParam&>(target.param);
			
			static double begin_pjs[6];
			static double step_pjs[6];

				// 获取当前起始点位置 //
				if (target.count == 1)
				{
					for (int i = 0; i < 6; ++i)
					{
						begin_pjs[i] = target.model->motionPool()[i].mp();
						step_pjs[i] = target.model->motionPool()[i].mp();
					}
				}
				for (int i = 0; i < 6; i++)
				{
					step_pjs[i] = begin_pjs[i] + 0.0001*target.count;
					target.model->motionPool().at(i).setMp(step_pjs[i]);
				}
			
			

			if (!target.model->solverPool().at(1).kinPos())return -1;

			// 访问主站 //
			auto controller = dynamic_cast<aris::control::Controller*>(target.master);

			// 打印电流 //
			auto &cout = controller->mout();
			if (target.count % 1000 == 0)
			{
				for (int i = 0; i < 6; i++)
				{
					cout << "pos" << i + 1 << ":" << target.model->motionPool()[i].mp() << "  ";
					cout << "vel" << i + 1 << ":" << target.model->motionPool()[i].mv() << "  ";
					cout << "cur" << i + 1 << ":" << target.model->motionPool()[i].ma() << "  ";
				}
				cout << std::endl;
			}

			// log 电流 //
			auto &lout = controller->lout();

			lout << target.model->motionPool()[0].mp() << ",";
			lout << target.model->motionPool()[1].mp() << ",";
			lout << target.model->motionPool()[2].mp() << ",";
			lout << target.model->motionPool()[3].mp() << ",";
			lout << target.model->motionPool()[4].mp() << ",";
			lout << target.model->motionPool()[5].mp() << ",";
			lout << std::endl;
			
			std::array<double, 6> RobotPosition;
			std::array<double, 6> RobotPositionT;
			std::array<double, 6> RobotPositionJ;
			std::array<double, 6> RobotVelocity;
			std::array<double, 6> RobotAcceleration;
			std::array<double, 6> TorqueSensor;
			std::array<double, 3> X1;
			std::array<double, 3> X2;
            
			std::array<double, 6> dX = { -0.0001, -0.0001, -0.0001, -0.0000, -0.0001, -0.0000};
			std::array<double, 6> dTheta;
			std::array<double, 6> estTorFin;
			std::array<double, 6> SumdTheta;
            //recvs[0] = 0; recvs[1] = 30; recvs[2] = 0; recvs[3] = 20; recvs[4] = 20; recvs[5] = 0;
            //recvs0[0] = 0; recvs0[1] = 30; recvs0[2] = 0; recvs0[3] = 20; recvs0[4] = 20; recvs0[5] = 0;
            int tick = 0;
            for (int i = 0; i < 6; i++)
                    estTorFin[i]=0;

  
                for (int i = 0; i < 6; i++)
                {
                    RobotPosition[i]=0;
                    RobotPositionT[i] = 0;
                    RobotPositionJ[i] = 0;
                    TorqueSensor[i] = 0;
                    RobotVelocity[i] = 0;
                    RobotAcceleration[i] = 0;
                }
				/*
                if (tick == 0)
                {
                    for (int j = 0; j < 6; j++)
                        stateTor0[j, 0] = TorqueSensor[j];
                    
                }

                for (int j = 0; j < 6; j++)
                {
                    double intDT = 0.008;
                    stateTor1[j, 0] = stateTor0[j, 0] + intDT * (A[0, 0] * stateTor0[j, 0] + A[0, 1] * stateTor0[j, 1] + A[0, 2] * stateTor0[j, 2] + B[0] * TorqueSensor[j]);
                    stateTor1[j, 1] = stateTor0[j, 1] + intDT * (A[1, 0] * stateTor0[j, 0] + A[1, 1] * stateTor0[j, 1] + A[1, 2] * stateTor0[j, 2] + B[1] * TorqueSensor[j]);
                    stateTor1[j, 2] = stateTor0[j, 2] + intDT * (A[2, 0] * stateTor0[j, 0] + A[2, 1] * stateTor0[j, 1] + A[2, 2] * stateTor0[j, 2] + B[2] * TorqueSensor[j]);
                }

                for (int j = 0; j < 6; j++)
                {
                   // TorqueSensor[j] = stateTor1[j, 0];
                }


                //estTor = distalDemo.distalCollision(RobotPosition, RobotVelocity, RobotAcceleration, TorqueSensor, estParas);

                //estTor[0] = -estTor[0];
                //estTorFin[2] = -estTorFin[2];
                //if (Math.Abs(estTor[0]) < 3)
                    //estTor[0] = 0;
                for (int i = 1; i < 6; i++)
                {
                    if(Math.Abs(estTor[i])<3)
                        estTor[i] = 0;
                }
                
              
                double FxListAve=0,FyListAve=0,FzListAve=0;
                
              
                for (int i = 0; i < 6; i++)
                {
                    X1[i] = estTor[i];
                    X2[i] = 0;
                }
                //RobotPosition[0] = -20; RobotPosition[1] = -8.42; RobotPosition[2] = 4.898;
                //RobotPosition[3] = -20; RobotPosition[4] = -8.42; RobotPosition[5] = 4.898;
                estTorFin = robotDemo.forceTransform(RobotPositionT, X1, X2);

                SumdTheta[9] = estTorFin[0];
                for (int i = 0; i < common.RobotAxis / 2; i++)
                {
                    estTorFin[i] = estTorFin[i];

                }
                
                for (int j = 0; j < common.RobotAxis / 2; j++)
                {
                    double intDT = 0.008;
                    stateDm1[j, 0] = stateDm0[j, 0] + intDT * (A[0, 0] * stateDm0[j, 0] + A[0, 1] * stateDm0[j, 1] + A[0, 2] * stateDm0[j, 2] + B[0] * estTorFin[j]);
                    stateDm1[j, 1] = stateDm0[j, 1] + intDT * (A[1, 0] * stateDm0[j, 0] + A[1, 1] * stateDm0[j, 1] + A[1, 2] * stateDm0[j, 2] + B[1] * estTorFin[j]);
                    stateDm1[j, 2] = stateDm0[j, 2] + intDT * (A[2, 0] * stateDm0[j, 0] + A[2, 1] * stateDm0[j, 1] + A[2, 2] * stateDm0[j, 2] + B[2] * estTorFin[j]);
                }

                dX[0, 0] = 1 * stateDm1[0, 0] / 5000;
                dX[1, 0] = 1 * stateDm1[1, 0] / 5000;
                dX[2, 0] = 0 * stateDm1[2, 0] / 20000;
                for (int i = 0; i < common.RobotAxis / 2; i++)
                {
                    
                    if (dX[i,0] > 0.0005)
                        dX[i, 0] = 0.0005;
                    if (dX[i, 0] < -0.0005)
                        dX[i, 0] = -0.0005;

                }

                //dX[0, 0] = FX.Value;
                //dX[1, 0] = FY.Value;
                //dX[2, 0] = FZ.Value;
                dX[3, 0] = MX.Value;
                dX[4, 0] = MY.Value;
                dX[5, 0] = MZ.Value;
				*/
               // dTheta = robotDemo.jointIncrement(RobotPositionJ, dX);
               
                for (int i = 0; i < 6; i++)
                {
                    dTheta[i] = dTheta[i] * ConAng * DirectionFlag[i];
 
                }
                
                
				double MaxdTheta = 0.03;
                for (int i = 1; i < 6; i++)
                {
                    if ( dTheta[i] > dTheta[i])
						dTheta[i] = dTheta[i];
					if (dTheta[i] < -dTheta[i])
						dTheta[i] = -dTheta[i];

                }
                

                
                for (int i = 0; i < 6; i++)
                {
                    //if (dTheta[i] > 0.3)
                    //    dTheta[i] = 0.3;
                   // if (dTheta[i] < -0.3)
                     //   dTheta[i] = -0.3;

                    SumdTheta[i] = SumdTheta[i] + dTheta[i];
                    if (SumdTheta[i] > 90)
                        SumdTheta[i] = 90;
                    if (SumdTheta[i] < -90)
                        SumdTheta[i] = -90;
                    //dqJ[j, i] = dTheta[i];
                }
               
               
      
                
                if (tick < 1000)
                    tick++;
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
            
				return 5000 - target.count;
}

MoveXYZ::MoveXYZ(const std::string &name) :Plan(name)
    {
        command().loadXmlStr(
            "<mvXYZ>"
			"	<group type=\"GroupParam\" default_child_type=\"Param\">"
			"		<damp default=\"1.0\" abbreviation=\"t\"/>"
			"	</group>"
            "</mvXYZ>");

		//mvXYZ --j1=0.1
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
#ifdef WIN32
		Plan::NOT_CHECK_POS_MIN |
		Plan::NOT_CHECK_POS_MAX |
		Plan::NOT_CHECK_POS_CONTINUOUS |
		Plan::NOT_CHECK_POS_CONTINUOUS_AT_START |
		Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
		Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER_AT_START |
		Plan::NOT_CHECK_POS_FOLLOWING_ERROR |
#endif
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
	int RecordNum = 1000;
	static double PositionList[1000][6];
	static double SensorList[1000][6];

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
	for (int i = 4; i < 6; i++)
	{
			step_pjs[i] = begin_pjs[i] + ampVar * (std::sin(2 * PI / param.period *target.count/1000));
			target.model->motionPool().at(i).setMp(step_pjs[i]);
	}
	
	if (!target.model->solverPool().at(1).kinPos())return -1;

	// 访问主站 //
	auto controller = dynamic_cast<aris::control::Controller*>(target.master);

	// 打印电流 //
	auto &cout = controller->mout();
	if (target.count % 1000 == 0)
	{
		for (int i = 0; i < 6; i++)
		{
			cout << "pos" << i + 1 << ":" << target.model->motionPool()[i].mp() << "  ";
			cout << "vel" << i + 1 << ":" << target.model->motionPool()[i].mv() << "  ";
			cout << "cur" << i + 1 << ":" << target.model->motionPool()[i].ma() << "  ";
		}
		cout << std::endl;
	}

    auto &lout = controller->lout();
	if(target.count<1000)
	{
		for (int i = 0; i < 6; i++)
		{
            PositionList[target.count][i] = target.model->motionPool()[i].mp();
			SensorList[target.count][i] = target.model->motionPool()[i].ma();
		}
		    lout << target.count << ",";
			lout << PositionList[target.count][0] << ",";lout << PositionList[target.count][1] << ",";
			lout << PositionList[target.count][2] << ",";lout << PositionList[target.count][3] << ",";
			lout << PositionList[target.count][4] << ",";lout << PositionList[target.count][5] << ",";
			lout << SensorList[target.count][0] << ",";lout << SensorList[target.count][1] << ",";
			lout << SensorList[target.count][2] << ",";lout << SensorList[target.count][3] << ",";
			lout << SensorList[target.count][4] << ",";lout << SensorList[target.count][5] << ",";

			lout << std::endl;
	}
	if (target.count == 1000)
		//estParas=sixDistalMatrix.RLS(PositionList, SensorList);

	return 5000 - target.count;
}

MoveDistal::MoveDistal(const std::string &name) :Plan(name)
{
	command().loadXmlStr(
		"<mvDistal>"
		"	<group type=\"GroupParam\" default_child_type=\"Param\">"
        "		<period default=\"2.0\" abbreviation=\"p\"/>"
        "		<amplitude default=\"0.1\" abbreviation=\"a\"/>"
		"	</group>"
		"</mvDistal>");
}









/*
auto MovePressure::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
{
    MoveCParam p;
   
}
auto MovePressure::executeRT(PlanTarget &target)->int
{
   
}

MovePressure::MovePressure(const std::string &name) :Plan(name)
    {
        command().loadXmlStr(
            "<mvEE>"
            "	<group type=\"GroupParam\">"
            "	    <total_time type=\"Param\" default=\"5000\"/>" //默认5000
            "       <radius type=\"Param\" default=\"0.01\"/>"
            "       <detal type=\"Param\" default=\"0.1/5000\"/>"//5秒走10cm
            "   </group>"
            "</mvEE>");
    }
	*/
