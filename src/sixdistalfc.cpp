#include "sixdistalfc.h"
#include <math.h>
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
double estParas[GroupDim];

double PositionList[6*SampleNum];
double SensorList[6 * SampleNum];

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
auto MoveXYZ::executeRT(PlanTarget &target)->int
{
			auto &param = std::any_cast<MoveXYZParam&>(target.param);
			
			double RobotPosition[6];
			double RobotPositionT[6];
			double RobotPositionJ[6];
			double RobotVelocity[6];
			double RobotAcceleration[6];
			double TorqueSensor[6];
			double X1[3];
			double X2[3];
			static double begin_pjs[6];
			static double step_pjs[6];
            static double stateTor0[6][3],stateTor1[6][3];
            static float FT0[7];
				// 获取当前起始点位置 //
				if (target.count == 1)
				{
                    for (int i = 0; i < 6; ++i)
					{
						step_pjs[i] = target.model->motionPool()[i].mp();
					}
				}
	

			if (!target.model->solverPool().at(1).kinPos())return -1;


			
			
            
            double dX[6] = { 0.00001, -0.0000, -0.0000, -0.0000, -0.0000, -0.0000};
            double dTheta[6]={0};
			double estTorFin[6];
			double SumdTheta[6];
            //recvs[0] = 0; recvs[1] = 30; recvs[2] = 0; recvs[3] = 20; recvs[4] = 20; recvs[5] = 0;
            //recvs0[0] = 0; recvs0[1] = 30; recvs0[2] = 0; recvs0[3] = 20; recvs0[4] = 20; recvs0[5] = 0;
            int tick = 0;
            for (int i = 0; i < 6; i++)
                    estTorFin[i]=0;

            float FT[7];
            uint16_t FTnum;
            auto conSensor = dynamic_cast<aris::control::EthercatController*>(target.master);
            conSensor->ecSlavePool().at(6).readPdo(0x6030, 0x00, &FTnum ,16);
            conSensor->ecSlavePool().at(6).readPdo(0x6030, 0x01, &FT[1] ,32);
            conSensor->ecSlavePool().at(6).readPdo(0x6030, 0x02, &FT[2], 32);
            conSensor->ecSlavePool().at(6).readPdo(0x6030, 0x03, &FT[3], 32);
            conSensor->ecSlavePool().at(6).readPdo(0x6030, 0x04, &FT[4], 32);
            conSensor->ecSlavePool().at(6).readPdo(0x6030, 0x05, &FT[5], 32);
            conSensor->ecSlavePool().at(6).readPdo(0x6030, 0x06, &FT[6], 32);
            // 获取当前起始点位置 //
            if (target.count == 1)
            {
                for (int i = 0; i < 7; ++i)
                {
                    FT0[i] = FT[i];
                }
                for (int j = 0; j < 6; j++)
                    stateTor0[j][0] = FT0[j+1];
            }







            for (int j = 0; j < 6; j++)
                {
                double A[3][3],B[3],CutFreq=10;
                A[0][0] = 0; A[0][1] = 1; A[0][2] = 0;
                A[1][0] = 0; A[1][1] = 0; A[1][2] = 1;
                A[2][0] = -CutFreq * CutFreq * CutFreq;
                A[2][1] = -2 * CutFreq * CutFreq;
                A[2][2] = -2 * CutFreq;
                B[0] = 0; B[1] = 0;
                B[2] = -A[2][0];
                 double intDT = 0.001;
                    stateTor1[j][0] = stateTor0[j][0] + intDT * (A[0][0] * stateTor0[j][0] + A[0][1] * stateTor0[j][1] + A[0][2] * stateTor0[j][2] + B[0] * FT[j+1]);
                    stateTor1[j][1] = stateTor0[j][1] + intDT * (A[1][0] * stateTor0[j][0] + A[1][1] * stateTor0[j][1] + A[1][2] * stateTor0[j][2] + B[1] * FT[j+1]);
                    stateTor1[j][2] = stateTor0[j][2] + intDT * (A[2][0] * stateTor0[j][0] + A[2][1] * stateTor0[j][1] + A[2][2] * stateTor0[j][2] + B[2] * FT[j+1]);
                }


               dX[0]=(stateTor1[2][0]-FT0[3])/150000;

               if(dX[0]>0.00035)
                   dX[0]=0.00035;
               if(dX[0]<-0.00035)
                   dX[0]=-0.00035;
               // 访问主站 //
               auto controller = dynamic_cast<aris::control::Controller*>(target.master);

               // 打印电流 //
               auto &cout = controller->mout();


               // log 电流 //
               auto &lout = controller->lout();
/*
               lout << target.model->motionPool()[0].mp() << ",";
               lout << target.model->motionPool()[1].mp() << ",";
               lout << target.model->motionPool()[2].mp() << ",";
               lout << target.model->motionPool()[3].mp() << ",";
               lout << target.model->motionPool()[4].mp() << ",";
               lout << target.model->motionPool()[5].mp() << ",";
               lout << FT[0] << ",";lout << FT[1] << ",";
               lout << FT[2] << ",";lout << FT[3] << ",";
               lout << FT[4] << ",";lout << FT[5] << ",";
               lout << stateTor0[0][0] << ",";lout << stateTor0[1][0] << ",";
               lout << stateTor0[2][0] << ",";lout << stateTor0[3][0] << ",";
               lout << stateTor0[4][0] << ",";lout << stateTor0[5][0] << ",";
*/
               //lout << stateTor1[2][0] << ",";lout << FT0[3] << ",";
              // lout << dX[0] << ",";

               lout << FTnum << ",";
               lout << FT[1] << ",";lout << FT[2] << ",";
               lout << FT[3] << ",";lout << FT[4] << ",";
               lout << FT[5] << ",";lout << FT[6] << ",";
               lout << std::endl;

               if (target.count % 1000 == 0)
               {
                   //for (int i = 0; i < 6; i++)
                   //{
                       //cout << "pos" << i + 1 << ":" << target.model->motionPool()[i].mp() << "  ";
                       //cout << "vel" << i + 1 << ":" << target.model->motionPool()[i].mv() << "  ";
                       //cout << "cur" << i + 1 << ":" << target.model->motionPool()[i].ma() << "  ";
                   //}

                   cout << "dX[0]" << FTnum<<"  ";
                   cout << std::endl;

               }
                for (int j = 0; j < 6; j++)
                {
                   // TorqueSensor[j] = stateTor1[j, 0];
                }

                /*
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
				for (int i = 0;i < 6;i++)
					RobotPositionJ[i] = target.model->motionPool()[i].mp();

                robotDemo.jointIncrement(RobotPositionJ, dX,dTheta);

                for (int i = 0; i < 6; i++)
                {
                    if (dTheta[i] > 0.03)
                        dTheta[i] = 0.03;
                    if (dTheta[i] < -0.03)
                       dTheta[i] = -0.03;
                }

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
            
                return 50000 - target.count;
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


int nn = 13; // n代表txt文档中数据的列数
vector<vector<double>> POSRLS(nn);
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

	for (int j = 0; j < nn; j++)
	{
		POSRLS[j].clear();
	}
	string filePath = "C:/Users/gk/Desktop/build_kaanh_gk/log/TestRLS.txt";

	ifstream oplog;
	oplog.open(filePath);
	if (!oplog)
	{
		cout << "fail to open the file" << endl;
		throw std::runtime_error("fail to open the file");
		//return -1;//或者抛出异常。
	}
	while (!oplog.eof())
	{
		for (int j = 0; j < nn; j++)
		{
			double data;
			oplog >> data;
			POSRLS[j].push_back(data);
		}
	}
	oplog.close();
	oplog.clear();
	for (int j = 0; j < nn; j++)
	{
		POSRLS[j].pop_back();
	}
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
    for (int i = 0; i < 6; i++)
	{
			step_pjs[i] = begin_pjs[i] + ampVar * (std::sin(2 * aris::PI / param.period *target.count/1000));
            target.model->motionPool().at(i).setMp(step_pjs[i]);
	}
	
    if (!target.model->solverPool().at(1).kinPos())return -1;

	// 访问主站 //
	auto controller = dynamic_cast<aris::control::Controller*>(target.master);

    float FT[7];
    int16_t FTnum;
    auto conSensor = dynamic_cast<aris::control::EthercatController*>(target.master);
    conSensor->ecSlavePool().at(6).readPdo(0x6030, 0x00, &FTnum ,16);
    conSensor->ecSlavePool().at(6).readPdo(0x6030, 0x01, &FT[1] ,32);
    conSensor->ecSlavePool().at(6).readPdo(0x6030, 0x02, &FT[2], 32);
    conSensor->ecSlavePool().at(6).readPdo(0x6030, 0x03, &FT[3], 32);
    conSensor->ecSlavePool().at(6).readPdo(0x6030, 0x04, &FT[4], 32);
    conSensor->ecSlavePool().at(6).readPdo(0x6030, 0x05, &FT[5], 32);
    conSensor->ecSlavePool().at(6).readPdo(0x6030, 0x06, &FT[6], 32);


	// 打印电流 //
	auto &cout = controller->mout();
    if (target.count % 10 == 0)
	{
        for (int i = 0; i < 6; i++)
        {
        	cout << "pos" << i + 1 << ":" << target.model->motionPool()[i].mp() << "  ";
        	cout << "vel" << i + 1 << ":" << target.model->motionPool()[i].mv() << "  ";
        	cout << "cur" << i + 1 << ":" << target.model->motionPool()[i].ma() << "  ";
        }
        //cout << FTnum << "  ";
		cout << std::endl;
	}

    auto &lout = controller->lout();
    //if(target.count<10000)
	{
		for (int i = 0; i < 6; i++)
		{
            PositionList[6*(target.count-1)+i] = POSRLS[i+1][target.count - 1];
            SensorList[6*(target.count-1)+i] = POSRLS[i+7][target.count - 1];
		}

        lout << FTnum << ",";
        lout << FT[1] << ",";lout << FT[2] << ",";
        lout << FT[3] << ",";lout << FT[4] << ",";
        lout << FT[5] << ",";lout << FT[6] << ",";
        /*
		    lout << target.count << ",";
			lout << PositionList[6 * (target.count - 1) + 0] << ",";lout << PositionList[6 * (target.count - 1) + 1] << ",";
			lout << PositionList[6 * (target.count - 1) + 2] << ",";lout << PositionList[6 * (target.count - 1) + 3] << ",";
			lout << PositionList[6 * (target.count - 1) + 4] << ",";lout << PositionList[6 * (target.count - 1) + 5] << ",";
			lout << SensorList[6 * (target.count - 1) + 0] << ",";lout << SensorList[6 * (target.count - 1) + 1] << ",";
			lout << SensorList[6 * (target.count - 1) + 2] << ",";lout << SensorList[6 * (target.count - 1) + 3] << ",";
			lout << SensorList[6 * (target.count - 1) + 4] << ",";lout << SensorList[6 * (target.count - 1) + 5] << ",";
*/
			lout << std::endl;
	}
    
		

    return SampleNum-target.count;
}

auto MoveDistal::collectNrt(aris::plan::PlanTarget &target)->void
{

	sixDistalMatrix.RLS(PositionList, SensorList, estParas);
	double a = 3;
}


MoveDistal::MoveDistal(const std::string &name) :Plan(name)
{
	command().loadXmlStr(
		"<mvDistal>"
		"	<group type=\"GroupParam\" default_child_type=\"Param\">"
        "		<period default=\"1.0\" abbreviation=\"p\"/>"
        "		<amplitude default=\"0.2\" abbreviation=\"a\"/>"
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
