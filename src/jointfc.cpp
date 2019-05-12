#include "jointfc.h"
#include <math.h>
#include <algorithm>
#include"robotconfig.h"
#include"jointdynamics.h"

using namespace std;
using namespace aris::plan;
using namespace aris::dynamic;
using namespace CONFIG;
using namespace JointDynamicsInt;
/// \brief

jointdynamics JointMatrix;


std::vector<double> AngleList_vec(6 * SampleNum);
auto AngleList = AngleList_vec.data();
std::vector<double> TorqueList_vec(6 * SampleNum);
auto TorqueList = TorqueList_vec.data();


struct JointDynaParam
{
	double period;
	double amplitude;

};
vector<vector<double>> POSRLS(13);
auto JointDyna::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
{
	JointDynaParam param;

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
auto JointDyna::executeRT(PlanTarget &target)->int
{
	auto &param = std::any_cast<JointDynaParam&>(target.param);
	static int CollectNum = 1;
	static double begin_pjs[RobotAxis];
	static double step_pjs[RobotAxis];
	static double perVar = 0;
	static double ampVar = 0;

	if (target.count < 1000)
	{
		ampVar = ampVar + param.amplitude / 1000;
	}
	// 获取当前起始点位置 //
	if (target.count == 1)
	{
		for (int i = 0; i < RobotAxis; i++)
		{
			begin_pjs[i] = target.model->motionPool()[i].mp();
			step_pjs[i] = target.model->motionPool()[i].mp();
		}
	}
	param.period = 60;



    static bool flag[6] = {true,true,true,true,true,true};
    double PosLimit[6] = { 1,0.5,0.3,1,1,1 };
    double NegLimit[6] = { -1,-0.5,-0.3,-1,-1,-1 };
    double dTheta = 0.00001;
	static double pArc[6], vArc[6], aArc[6], vArcMax[6] = { 0.15,0.15,0.15,0.15,0.15,0.15 };
	static aris::Size t_count[6] = { 0 };

	static int CountOffsetPos[6] = { 1,1,1,1,1,1 }, CountOffsetNeg[6] = { 1,1,1,1,1,1 };


	for (int i = 0;i < 6;i++)
	{

		if (flag[i])
		{
			if (step_pjs[i] < PosLimit[i])
			{
				aris::plan::moveAbsolute(target.count - CountOffsetNeg[i] + 1, 0, PosLimit[i] - begin_pjs[i], vArcMax[i] / 1000, 0.05 / 1000 / 1000, 0.05 / 1000 / 1000, pArc[i], vArc[i], aArc[i], t_count[i]);

				step_pjs[i] = step_pjs[i] + vArc[i];
			}
			//std::cout << vArc << "  ";
			if ((t_count[i] - (target.count - CountOffsetNeg[i] + 1)) < 0.5 && (t_count[i] - (target.count - CountOffsetNeg[i] + 1)) > -0.5)
			{
				CountOffsetPos[i] = target.count;
				flag[i] = false;
				begin_pjs[i] = target.model->motionPool()[i].mp();
			}


		}
		if (flag[i] == false)
		{
			if (step_pjs[i] > NegLimit[i])
			{
				aris::plan::moveAbsolute(target.count - CountOffsetPos[i] + 1, 0, begin_pjs[i] - NegLimit[i], vArcMax[i] / 1000, 0.05 / 1000 / 1000, 0.05 / 1000 / 1000, pArc[i], vArc[i], aArc[i], t_count[i]);

				step_pjs[i] = step_pjs[i] - vArc[i];
			}

			if ((t_count[i] - (target.count - CountOffsetPos[i] + 1)) < 0.5 && (t_count[i] - (target.count - CountOffsetPos[i] + 1)) > -0.5)
			{
				CountOffsetNeg[i] = target.count;
				flag[i] = true;
				begin_pjs[i] = target.model->motionPool()[i].mp();
			}

		}
			target.model->motionPool().at(i).setMp(step_pjs[i]);
	}



	if (!target.model->solverPool().at(1).kinPos())return -1;

	// 访问主站 //
	auto controller = target.controller;

	

	// 打印电流 //
	auto &cout = controller->mout();
	if (target.count % 100 == 0)
	{
		//for (int i = 0; i < 6; i++)
		{
			cout << step_pjs[4] << "***"<< "  ";
			//cout << "vel" << i + 1 << ":" << target.model->motionPool()[i].mv() << "  ";
			//cout << "cur" << i + 1 << ":" << target.model->motionPool()[i].ma() << "  ";
		}
		//   cout << target.count << "  ";
		cout << std::endl;
	}

	auto &lout = controller->lout();

	double f2c_index[6] = { 9.07327526291993, 9.07327526291993, 17.5690184835913, 39.0310903520972, 66.3992503259041, 107.566785527965 };
	if (target.count % 8 == 0)
	{
		for (int i = 0; i < 6; i++)
		{
			//AngleList[RobotAxis * (target.count - 1) + i] = controller->motionAtAbs(i).actualPos();
			//TorqueList[RobotAxis * (target.count - 1) + i] = controller->motionAtAbs(i).actualCur();

			//AngleList[6 * (target.count - 1) + i] = POSRLS[i][target.count - 1];
			//TorqueList[6 * (target.count - 1) + i] = POSRLS[i + 6][target.count - 1];

			AngleList[6 * (CollectNum - 1) + i] = controller->motionAtAbs(i).actualPos();
			TorqueList[6 * (CollectNum - 1) + i] = controller->motionAtAbs(i).actualCur() / f2c_index[i];
		}

		lout << target.count << ",";
		lout << AngleList[RobotAxis * (CollectNum - 1) + 0] << ",";lout << AngleList[RobotAxis * (CollectNum - 1) + 1] << ",";
		lout << AngleList[RobotAxis * (CollectNum - 1) + 2] << ",";lout << AngleList[RobotAxis * (CollectNum - 1) + 3] << ",";
		lout << AngleList[RobotAxis * (CollectNum - 1) + 4] << ",";lout << AngleList[RobotAxis * (CollectNum - 1) + 5] << ",";
		lout << TorqueList[RobotAxis * (CollectNum - 1) + 0] << ",";lout << TorqueList[RobotAxis * (CollectNum - 1) + 1] << ",";
		lout << TorqueList[RobotAxis * (CollectNum - 1) + 2] << ",";lout << TorqueList[RobotAxis * (CollectNum - 1) + 3] << ",";
		lout << TorqueList[RobotAxis * (CollectNum - 1) + 4] << ",";lout << TorqueList[RobotAxis * (CollectNum - 1) + 5] << ",";

		lout << std::endl;
		CollectNum = CollectNum + 1;
	}


	return SampleNum - CollectNum;
}


auto JointDyna::collectNrt(aris::plan::PlanTarget &target)->void
{
	
	double StatisError[6] = { 0,0,0,0,0,0 };
	//  auto controller = target.controller;
	 // auto &lout = controller->lout();
	std::cout << "collect" << std::endl;


	JointMatrix.RLS(AngleList, TorqueList, JointMatrix.estParasJoint, JointMatrix.CoefParasJoint,StatisError);

	
	//std::cout<<"collect"<<std::endl;
	for (int i = 0;i < JointReduceDim+12;i++)
		cout << JointMatrix.estParasJoint[i] << ",";

	/*
	//Save Estimated Paras
	ofstream outfile("C:/Users/gk/Desktop/Kaanh_gk/EstParas.txt");
	if (!outfile)
	{
		cout << "Unable to open otfile";
		exit(1); // terminate with error
	}

	for (int k = 0; k < JointGroupDim; k++)
	{
		outfile << estParas[k] << "   " << std::endl;
		//cout << "success outfile " << endl;
	}
	outfile.close();
	*/

	std::cout << endl;
	std::cout << "*****************************Statictic Model Error*****************************************" << std::endl;
	for (int i = 0;i < RobotAxis;i++)
		cout << StatisError[i] << endl;
	//double a = 3;
}


JointDyna::JointDyna(const std::string &name) :Plan(name)
{

	command().loadXmlStr(
		"<Command name=\"JointDyna\">"
		"	<GroupParam>"
		"		<Param name=\"period\"default=\"20.0\"/>"
		"		<Param name=\"amplitude\" default=\"0.2\"/>"
		"	</GroupParam>"
		"</Command>");

}


struct LoadDynaParam
{
	double period;
	double amplitude;
	bool InitEst;

};

auto LoadDyna::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
{
	LoadDynaParam param;

	for (auto &p : params)
	{
		if (p.first == "period")
			param.period = std::stod(p.second);
		if (p.first == "amplitude")
			param.amplitude = std::stod(p.second);
		if (p.first == "InitEst")
			param.InitEst = std::stod(p.second);

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

	/*
	int nn = 12; // n代表txt文档中数据的列数


	for (int j = 0; j < nn; j++)
	{
		POSRLS[j].clear();
	}
	string filePath = "C:/Users/gk/Desktop/build_kaanh_gk/test1.txt";

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

	

	// 所需的中间变量，请对U的对角线元素做处理
	double TestQR[6] = {1.1,2.1,3.31,4.121,5.81,9.31};
	
	double U[6] = {0};
	double TestQ[9] = { 0 };
	double TestR[6] = { 0 };
	double tau[3];
	aris::Size p[3];
	aris::Size rank;


	// 根据 A 求出中间变量，相当于做 QR 分解 //
// 请对 U 的对角线元素做处理
	s_householder_utp(3, 2, TestQR, U, tau, p, rank, 1e-10);
	s_householder_ut2q(3, 2, U, tau, TestQ);
	s_householder_ut2r(3, 2, U, tau, TestR);


	//s_permutate_inv(2, 3,  p, TestR, T(2));
	//s_permutate(2, rhs,  p, x,x_t);
	//s_householder_ut2qmn(3, 3, U, tau, TestQ);
	s_mm(3, 2, 3, TestQ, TestR, U);
	int s = 4;*/
}
auto LoadDyna::executeRT(PlanTarget &target)->int
{
	auto &param = std::any_cast<LoadDynaParam&>(target.param);

    static int CollectNum=1;
	static double begin_pjs[RobotAxis];
	static double step_pjs[RobotAxis];
	static double perVar = 0;
	static double ampVar = 0;

	if (target.count < 1000)
	{
		ampVar = ampVar + param.amplitude / 1000;
	}
	// 获取当前起始点位置 //
	if (target.count == 1)
	{
		for (int i = 0; i < RobotAxis; i++)
		{
			begin_pjs[i] = target.model->motionPool()[i].mp();
			step_pjs[i] = target.model->motionPool()[i].mp();
		}
	}

	static bool flag[6] = {true,true,true,true,true,true};
    double PosLimit[6] = { 1,1,0.2,1,0.5,1 };
    double NegLimit[6] = { -1,-1,-0.2,-1,-0.5,-1 };
	double dTheta = 0.0001;
	static double pArc[6], vArc[6], aArc[6], vArcMax[6] = { 0.15,0.15,0.15,0.15,0.15,0.15 };
	static aris::Size t_count[6] = { 0 };
	
	static int CountOffsetPos[6] = { 1,1,1,1,1,1 }, CountOffsetNeg[6] = { 1,1,1,1,1,1};

	for (int i = 0;i < 6;i++)
	{
		
		if (flag[i])
		{
			if (step_pjs[i] < PosLimit[i])
			{
				aris::plan::moveAbsolute(target.count - CountOffsetNeg[i] + 1, 0, PosLimit[i] - begin_pjs[i], vArcMax[i] / 1000, 0.05 / 1000 / 1000, 0.05 / 1000 / 1000, pArc[i], vArc[i], aArc[i], t_count[i]);

				step_pjs[i] = step_pjs[i] + vArc[i];
			}
				//std::cout << vArc << "  ";
				if ((t_count[i]-(target.count - CountOffsetNeg[i] + 1))<0.5&& (t_count[i] - (target.count - CountOffsetNeg[i] + 1)) > -0.5)
				{
					CountOffsetPos[i] = target.count;
					flag[i] = false;
					begin_pjs[i] = target.model->motionPool()[i].mp();
				}
				
			
		}
		if (flag[i] == false)
		{
			if (step_pjs[i] > NegLimit[i])
			{
				aris::plan::moveAbsolute(target.count - CountOffsetPos[i] + 1, 0, begin_pjs[i] - NegLimit[i], vArcMax[i] / 1000, 0.05 / 1000 / 1000, 0.05 / 1000 / 1000, pArc[i], vArc[i], aArc[i], t_count[i]);

				step_pjs[i] = step_pjs[i] - vArc[i];
			}

			if ((t_count[i] - (target.count - CountOffsetPos[i] + 1)) < 0.5 && (t_count[i] - (target.count - CountOffsetPos[i] + 1)) > -0.5)
			{
				CountOffsetNeg[i] = target.count;
				flag[i] = true;
				begin_pjs[i] = target.model->motionPool()[i].mp();
			}
			
		}
		if(i==2||i==4||i==5)
            target.model->motionPool().at(i).setMp(step_pjs[i]);
	}






	if (!target.model->solverPool().at(1).kinPos())return -1;

	// 访问主站 //
	auto controller = target.controller;



	// 打印电流 //
	auto &cout = controller->mout();
	if (target.count % 100 == 0)
	{
		//for (int i = 0; i < 6; i++)
		{
			//cout << step_pjs[4] << "***" << "  ";
			//cout << "vel" << i + 1 << ":" << target.model->motionPool()[i].mv() << "  ";
			//cout << "cur" << i + 1 << ":" << target.model->motionPool()[i].ma() << "  ";
		}
		  cout << vArc[2]*1000 << "  "<< step_pjs[2]<<" "<<flag[2]<<" "<<t_count[2]<<" "<<target.count - (CountOffsetNeg[2] - 1)<<" ";
		cout << std::endl;
	}

	auto &lout = controller->lout();

	double f2c_index[6] = { 9.07327526291993, 9.07327526291993, 17.5690184835913, 39.0310903520972, 66.3992503259041, 107.566785527965 };

	

    if(target.count%8==0)
    {
        for (int i = 0; i < 6; i++)
        {
            //AngleList[RobotAxis * (target.count - 1) + i] = controller->motionAtAbs(i).actualPos();
            //TorqueList[RobotAxis * (target.count - 1) + i] = controller->motionAtAbs(i).actualCur();

            //AngleList[6 * (target.count - 1) + i] = POSRLS[i][target.count - 1];
            //TorqueList[6 * (target.count - 1) + i] = POSRLS[i + 6][target.count - 1];

            AngleList[6 * (CollectNum - 1) + i] = controller->motionAtAbs(i).actualPos();
            TorqueList[6 * (CollectNum - 1) + i] = controller->motionAtAbs(i).actualCur() / f2c_index[i];
        }
	
        lout << target.count << ",";
        lout << AngleList[RobotAxis * (CollectNum - 1) + 0] << ",";lout << AngleList[RobotAxis * (CollectNum - 1) + 1] << ",";
        lout << AngleList[RobotAxis * (CollectNum - 1) + 2] << ",";lout << AngleList[RobotAxis * (CollectNum - 1) + 3] << ",";
        lout << AngleList[RobotAxis * (CollectNum - 1) + 4] << ",";lout << AngleList[RobotAxis * (CollectNum - 1) + 5] << ",";
        lout << TorqueList[RobotAxis * (CollectNum - 1) + 0] << ",";lout << TorqueList[RobotAxis * (CollectNum - 1) + 1] << ",";
        lout << TorqueList[RobotAxis * (CollectNum - 1) + 2] << ",";lout << TorqueList[RobotAxis * (CollectNum - 1) + 3] << ",";
        lout << TorqueList[RobotAxis * (CollectNum - 1) + 4] << ",";lout << TorqueList[RobotAxis * (CollectNum - 1) + 5] << ",";

        lout << std::endl;
        CollectNum=CollectNum+1;
    }
	

    return SampleNum - CollectNum;
}


auto LoadDyna::collectNrt(aris::plan::PlanTarget &target)->void
{
	auto &param = std::any_cast<LoadDynaParam&>(target.param);
	double dEst[LoadTotalParas] = { 0 };
	double dCoef[LoadReduceParas*10] = { 0 };
	double StatisError[3] = { 0,0,0};
	//  auto controller = target.controller;
	 // auto &lout = controller->lout();
    std::cout << "param.InitEst" << std::endl;

	

		JointMatrix.LoadRLS(AngleList, TorqueList, JointMatrix.estParasL0, JointMatrix.CoefParasL0,StatisError);
        std::cout << "collect" << std::endl;
    /*
	else
	{
		JointMatrix.LoadRLS(AngleList, TorqueList, JointMatrix.estParasL, JointMatrix.CoefParasL,StatisError);
		for (int i = 0;i < LoadReduceParas;i++)
			dEst[i] = JointMatrix.estParasL[i] - JointMatrix.estParasL0[i];

		for (int i = 0;i < LoadReduceParas;i++)
			for (int j = 0;j < 10;j++)
				dCoef[i * 10 + j] = JointMatrix.CoefParasL[i*LoadTotalParas+30+j] - JointMatrix.CoefParasL0[i * LoadTotalParas + 30 + j];

		JointMatrix.LoadParasExt(dEst,dCoef, JointMatrix.LoadParas);


    }*/
	


	//std::cout<<"collect"<<std::endl;
	for (int i = 0;i < LoadReduceParas+6;i++)
        cout << JointMatrix.estParasL0[i] << ",";

	/*
	//Save Estimated Paras
	ofstream outfile("C:/Users/gk/Desktop/Kaanh_gk/EstParas.txt");
	if (!outfile)
	{
		cout << "Unable to open otfile";
		exit(1); // terminate with error
	}

	for (int k = 0; k < JointGroupDim; k++)
	{
		outfile << estParas[k] << "   " << std::endl;
		//cout << "success outfile " << endl;
	}
	outfile.close();
	*/

	std::cout << endl;
	std::cout << "*****************************Statictic Model Error*****************************************" << std::endl;
	for (int i = 0;i < 3;i++)
		cout << StatisError[i] << endl;
	//double a = 3;
}


LoadDyna::LoadDyna(const std::string &name) :Plan(name)
{

	command().loadXmlStr(
		"<Command name=\"LoadDyna\">"
		"	<GroupParam>"
		"		<Param name=\"period\"default=\"20.0\"/>"
		"		<Param name=\"amplitude\" default=\"0.2\"/>"
		"	</GroupParam>"
		"</Command>");

}
