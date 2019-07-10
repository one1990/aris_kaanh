﻿#include "jointfc.h"
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

 for(auto &option:target.mot_options) option|=
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
    double PosLimit[6] = { 1,0.51,0.5,1,1,1 };
    double NegLimit[6] = { -1,-0.5,-0.5,-1,-1,-1 };
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


    if (target.model->solverPool().at(1).kinPos()) return -1;


	// 访问主站 //
	auto controller = target.controller;

	

	// 打印电流 //
	auto &cout = controller->mout();
	if (target.count % 100 == 0)
	{
        for (int i = 0; i < 6; i++)
		{
            //cout << controller->motionAtAbs(i).actualTor() << "***"<< "  ";
			//cout << "vel" << i + 1 << ":" << target.model->motionPool()[i].mv() << "  ";
			//cout << "cur" << i + 1 << ":" << target.model->motionPool()[i].ma() << "  ";
		}
		//   cout << target.count << "  ";
		cout << std::endl;
	}

	auto &lout = controller->lout();

	double f2c_index[6] = { 9.07327526291993, 9.07327526291993, 17.5690184835913, 39.0310903520972, 66.3992503259041, 107.566785527965 };
    double f2f_index[6]={129.6*1.27/1000, -100*2.39/1000, 101*1.27/1000, 81.6*0.318/1000, 81*0.318/1000, 51*0.318/1000};

    if (target.count % 8 == 0)
	{
		for (int i = 0; i < 6; i++)
		{
			//AngleList[RobotAxis * (target.count - 1) + i] = controller->motionAtAbs(i).actualPos();
			//TorqueList[RobotAxis * (target.count - 1) + i] = controller->motionAtAbs(i).actualCur();

            //AngleList[6 * (target.count - 1) + i] = POSRLS[i][target.count - 1];
            //TorqueList[6 * (target.count - 1) + i] = POSRLS[i + 6][target.count - 1] / f2c_index[i];

            AngleList[6 * (CollectNum - 1) + i] = controller->motionAtAbs(i).actualPos();
            TorqueList[6 * (CollectNum - 1) + i] = controller->motionAtAbs(i).actualCur() / f2c_index[i];
            //TorqueList[6 * (CollectNum - 1) + i] = 2;//controller->motionAtAbs(i).actualTor() * f2f_index[i];
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


    JointMatrix.RLSaris(AngleList, TorqueList, JointMatrix.estParasJoint, JointMatrix.CoefParasJoint, JointMatrix.CoefParasJointInv, StatisError);

	
	//std::cout<<"collect"<<std::endl;
	for (int i = 0;i < JointReduceDim+12;i++)
		cout << JointMatrix.estParasJoint[i] << ",";

	aris::core::Matrix mat0(1,JointReduceDim + 12, JointMatrix.estParasJoint);
	if (target.model->variablePool().findByName("estParasJoint") !=
		target.model->variablePool().end())
	{
		dynamic_cast<aris::dynamic::MatrixVariable*>(
			&*target.model->variablePool().findByName("estParasJoint"))->data() = mat0;
	}
	else
	{
		target.model->variablePool().add<aris::dynamic::MatrixVariable>("estParasJoint", mat0);
	}

	aris::core::Matrix mat1(1,JointReduceDim*JointGroupDim, JointMatrix.CoefParasJoint);
	if (target.model->variablePool().findByName("CoefParasJoint") !=
		target.model->variablePool().end())
	{
		dynamic_cast<aris::dynamic::MatrixVariable*>(
			&*target.model->variablePool().findByName("CoefParasJoint"))->data() = mat1;
	}
	else
	{
		target.model->variablePool().add<aris::dynamic::MatrixVariable>("CoefParasJoint", mat1);
	}
	
	aris::core::Matrix mat2(1,JointReduceDim*JointGroupDim, JointMatrix.CoefParasJointInv);
	if (target.model->variablePool().findByName("CoefParasJointInv") !=
		target.model->variablePool().end())
	{
		dynamic_cast<aris::dynamic::MatrixVariable*>(
			&*target.model->variablePool().findByName("CoefParasJointInv"))->data() = mat2;
	}
	else
	{
		target.model->variablePool().add<aris::dynamic::MatrixVariable>("CoefParasJointInv", mat2);
	}


	std::cout << endl;
	std::cout << "*****************************Statictic Model Error*****************************************" << std::endl;
	for (int i = 0;i < RobotAxis;i++)
		cout << StatisError[i] << endl;
	/*
	测试外力检测函数
	double A[1800] = { 0,0,0,0,0,-0.00081676,0,0,0,0,-0.03675,0.27795,-0.30261,0.035933,0.033889,0.19495,-17.8114,-286.2862,2.4757e-19,-1.3068e-06,-0.10963,0.29835,-0.57271,0.10882,-0.013418,1.0498,-34.8127,-526.7709,0.083217,-4.8807,0.083175,0.54116,-0.033679,0.85698,0.56527,0.77169,-9.9906,47.359,-526.2518,-153.1745,0.091267,0.28519,-0.02028,0.7636,-0.076578,0.26292,-287.3715,-372.2751,-47.359,-153.1745,-0.31133,0.21971,-0.043575,0.66552,0.52075,1.2598,-152.6009,240.0921,-372.2751,-153.1745,0,0,0,0,0,0.0022283,0,0,0,0,-0.052857,0.02219,-0.084998,0.055085,0.061386,0.19189,31.7858,0.6415,-1.398e-18,3.5653e-06,-0.11268,0.04792,-0.14888,0.11491,0.20394,0.33018,72.7705,0,0.023374,8.7598,-0.23564,0.084075,-0.33982,0.45314,0.23242,0.6711,198.6538,135.8771,0.1206,10.596,-0.55618,0.2248,-0.41136,0.99164,0.28292,0.83462,301.5805,-122.7674,-135.8771,10.596,-0.038553,1.9296,-1.1365,0.31699,0.54723,1.8669,465.9836,-36.4325,-122.7674,10.596,0,0,0,0,0,0.00078312,0,0,0,0,0.10445,-0.070782,-0.21565,-0.10367,0.10272,-0.72177,-65.935,-2.4401,1.1598e-19,1.253e-06,0.2622,-0.12085,-0.38636,-0.26142,0.28847,-2.7936,-189.6583,0,0.059305,-18.1946,-0.51455,-2.4303,0.99043,-2.0169,0.50063,-0.56648,-102.2671,224.5087,-1.4146,-23.1355,0.13593,0.088039,1.3905,-1.217,-2.8336,-2.749,-212.7179,90.5444,-224.5087,-23.1355,-1.5672,-2.6178,3.8575,-1.0459,2.7443,-2.7646,0,460.9902,90.5444,-23.1355,0,0,0,0,0,-0.0037966,0,0,0,0,-0.041272,0.034733,0.17079,0.037475,-0.087439,0.34604,25.772,1.3119,6.5661e-19,-6.0745e-06,-0.10739,0.056649,0.30419,0.10359,-0.25837,1.4799,80.47,0,-0.046968,7.1163,0.77651,1.5447,-0.32052,0.59597,-0.19022,0.53079,136.7573,25.949,0.76718,9.2362,-0.26898,-0.48396,-2.2924,1.5763,2.3786,2.5496,58.9788,-324.9611,-25.949,9.2362,1.4578,2.6942,-3.2523,0.82287,-0.36415,2.4393,0,0,-324.9611,9.2362,0,0,0,0,0,9.3858e-05,0,0,0,0,-0.027655,0.29132,-0.28529,0.027749,0.041775,0.12722,-21.1295,-287.6363,-9.7788e-20,1.5017e-07,0.0066117,-0.005245,0.021508,-0.0065178,0.017158,-0.10827,-3.7186,0,0.078456,-5.7989,-0.063393,0.0092169,-0.043468,-0.038266,-0.012482,0.015778,-1.9893,2.2994,-0.056798,-5.8999,-0.024277,0.12285,-0.0072787,-0.023338,0.02383,-0.038796,-1.5811,0,-2.2994,-5.8999,0.055551,0.10932,0.05086,-0.11862,-0.054087,0.044723,0,0,4.3243e-16,-5.8999,0,0,0,0,0,0.0029699,0,0,0,0,-0.11456,0.080145,-0.24481,0.11753,0.011394,0.53882,76.2929,0,-1.4474e-19,4.7519e-06,-0.3054,0.14874,-0.48281,0.30837,-0.033124,2.3987,233.1646,0,0.067324,21.0302,0.91789,2.4318,-0.25456,1.1754,1.4072,0.094704,159.5591,-106.7923,1.1685,27.0241,0.27032,-0.82688,-2.036,0.74228,-0.2597,-0.76289,85.1186,0,106.7923,27.0241,0.64414,-0.45705,0.089337,-1.1367,3.0878,-0.015031,0,0,1.7918e-15,27.0241,0,0,0,0,0,-0.0018788,0,0,0,0,-0.0011067,-0.0050728,-0.06908,-0.00077217,0.013362,-0.034175,-0.71234,0,4.0666e-19,-3.0061e-06,0.00013488,0.00040674,-0.12832,-0.0020137,0.025484,-0.094153,0,0,0.018997,-0.19854,-0.2262,-0.34276,-0.1467,0.13218,-0.60091,-0.15587,-113.4265,-197.3378,-0.05266,-0.20597,-0.13593,-0.06829,0.25106,-0.24614,0.20845,0.36581,-66.6316,0,197.3378,-0.20597,-0.0075327,0.9465,0.019087,0.23741,1.2606,-0.84163,0,0,-3.2201e-15,-0.20597,0,0,0,0,0,-0.0031925,0,0,0,0,0.0049995,-0.0067527,-0.22577,-0.008192,0.057171,-0.159,-5.7725,0,9.5025e-19,-5.108e-06,-0.0020228,-0.021609,-0.42788,-0.0011697,0.13054,-0.23556,0,0,0.062088,-1.6001,0.35872,0.14566,0.22974,-0.5963,0.024482,0.10104,18.718,-4.1609e-16,-0.1325,-1.6187,0.65694,-0.010062,-0.13188,-0.19718,-0.40119,-1.9218,-183.9037,0,0,-1.6187,0.48303,-1.6103,0.43929,-1.7479,-0.41417,0.79141,0,0,-5.1262e-16,-1.6187,0,0,0,0,0,0.0011044,0,0,0,0,0.17866,-0.12168,-0.10717,-0.17755,0.058617,-1.0104,-112.7994,0,-2.7444e-19,1.767e-06,0.0034508,-0.010186,-0.12266,-0.0023464,-0.020449,0.10873,0,0,0.029471,-31.1097,0.35308,0.35531,0.1679,-0.24089,0.1179,-0.0089368,3.0565,-3.3238e-16,0.063078,-31.1007,0.23053,-0.065526,-0.20249,0.11361,0.21674,-0.2343,0,0,0,-31.1007,0.28932,-0.28277,0.1439,-0.29309,0.0099106,0.039262,0,0,1.2306e-16,-31.1007,0,0,0,0,0,0.0024,0,0,0,0,0.0031258,0.0047331,0.16914,-0.0007258,-0.042817,0.091163,0,0,1.1418e-20,3.84e-06,0.0011077,0.016086,0.31714,0.0012923,-0.10951,0.26018,0,0,-0.046514,0.0068432,-0.43193,0.17343,0.10562,0.69322,1.3098,-1.0121,-112.3188,5.1816e-16,0.14592,0.027379,-1.0519,-0.67828,-1.7965,-0.39222,-1.3001,2.3802,0,0,0,0.027379,1.4828,3.3143,-2.1145,-0.15444,1.442,-0.32269,0,0,2.1188e-15,0.027379,0,0,0,0,0,0.23536,0,0,0,0,0.2369,-0.49051,0.40267,-0.0015439,-0.16868,0.29021,0,0,-2.4369e-16,0.00037658,0.29307,-0.58133,0.975,-0.057705,0.20072,1.0561,0,0,-0.11073,0.022207,-0.90058,1.0621,-1.5037,2.2498,-0.4133,-0.74546,0,4.3766e-16,0.77009,0.13268,-1.344,0.30954,-0.9955,-0.30202,-0.53689,3.9463,0,0,0,0.13268,1.2442,9.7242,-0.59847,1.3581,-0.043173,-0.90127,0,0,-3.6104e-16,0.13268,0,0,0,0,0,-0.41244,0,0,0,0,-0.48514,-0.15117,-0.37465,0.072703,-0.33122,0.17298,0,0,4.6524e-17,-0.0006599,-0.39708,-0.53287,-0.24339,-0.015363,-0.88236,0.77276,0,0,0.10303,0.01792,0.1412,3.188,0.35868,0.23448,2.2425,-0.18878,0,5.694e-16,0.2237,0.051577,0.11299,-0.27473,-6.7479,-0.16057,0.58217,-0.89964,0,0,0,0.051577,3.5641,0,0.20897,-4.3507,1.7917,-0.1148,0,0,-1.618e-15,0.051577,0,0,0,0,0,-0.20423,0,0,0,0,-0.20498,0.36627,0.32013,0.00074328,0.26767,0.25955,0,0,-5.5545e-17,-0.00032677,-0.33517,0.58835,0.38583,0.13094,0.33159,0.9385,0,0,-0.088035,0.019358,0.1235,0.88238,-0.49374,0.47982,-1.8165,0.67665,0,-1.2503e-16,0.32315,0.063209,0.25456,0.57573,0,0.5456,3.6849,1.3815,0,0,0,0.063209,1.3814,0,-2.0283,0.25461,-4.8882,1.7187,0,0,2.5104e-15,0.063209,0,0,0,0,0,0.24234,0,0,0,0,0.19703,-0.13594,0.39544,0.045306,-0.071749,0.37119,0,0,5.7089e-17,0.00038774,0.076603,-0.072824,0.58262,0.16573,-0.27924,1.3822,0,0,-0.10874,0.031885,-0.039673,-0.21384,0.84888,1.4985,0.1864,0.18867,0,-2.0871e-16,0.81875,0.14773,0.56459,-0.78925,0,-0.4156,-0.77796,2.3801,0,0,0,0.14773,-1.3903,0,0.093708,4.335,0,-0.89565,0,0,1.8518e-15,0.14773,0,0,0,0,0,-0.1895,0,0,0,0,-0.093673,-0.30577,0.33657,-0.095826,-0.14906,0.18562,0,0,-1.0932e-16,-0.0003032,-0.018498,-0.25062,0.88607,-0.171,0.0018272,0.66743,0,0,-0.092558,0.0064873,0.23517,0.59395,-0.73807,0.41377,-0.84885,-0.31672,0,1.337e-16,0.36967,0.059429,-0.27766,-1.3641,0,0.19611,1.3543,0.32697,0,0,0,0.059429,0.049303,0,-4.1712,0,0,0.83578,0,0,-1.5875e-15,0.059429,0,0,0,0,0,0.39428,0,0,0,0,0.36347,-0.59087,1.754,0.030805,-0.36514,0.055771,0,0,1.8946e-16,0.00063084,0.22685,-0.68991,3.4674,0.16742,-1.0212,0.15113,0,0,-0.48235,0.0071781,-1.1168,-0.56457,-0.078965,1.4947,0.46406,-0.088038,0,1.8638e-17,0.22892,0.041841,-0.28663,0.41395,0,-0.91817,-0.4781,1.3054,0,0,0,0.041841,1.0188,0,0,0,0,-1.2902,0,0,-3.7616e-16,0.041841,0,0,0,0,0,-0.13995,0,0,0,0,-0.089989,0.47376,0.065035,-0.049964,-0.10922,0.093648,0,0,2.0876e-16,-0.00022392,0.048135,0.40431,0,-0.18809,-0.46359,0.45818,0,0,-0.017885,0.0030797,0.41947,0.4721,0.4269,0.086843,0.43653,0.42307,0,5.6629e-16,0.27343,0.040113,0.0040086,0.79888,0,0.83853,0.57883,-0.37633,0,0,0,0.040113,-0.37232,0,0,0,0,3.0295,0,0,-1.6156e-16,0.040113,0,0,0,0,0,0.19593,0,0,0,0,0.11127,-0.31954,-0.036464,0.084656,-0.015593,-0.13578,0,0,-1.1558e-16,0.00031348,0.033055,-0.37002,0,0.16287,-0.033457,-0.75514,0,0,0.010028,-0.0035531,-0.85112,-0.98901,0.03718,0.12903,1.0291,-0.19095,0,8.1844e-17,-0.39512,-0.057945,-0.46898,0.061079,0,-0.57309,-2.6042,-0.22417,0,0,0,-0.057945,-0.69314,0,0,0,0,0,0,0,-6.1064e-16,-0.057945,0,0,0,0,0,-0.74894,0,0,0,0,-0.6071,0.17108,-0.1584,-0.14185,0.1087,-0.069807,0,0,3.794e-17,-0.0011983,-0.47942,0.14989,0,-0.26952,0.54825,-0.43658,0,0,0.043561,-0.017205,-1.3354,-0.38888,-1.4184,0.41938,-1.2094,-0.28413,0,1.4132e-16,-0.51671,-0.09051,-1.1741,0.094095,0,-0.44539,0,-1.1774,0,0,0,-0.09051,-2.3515,0,0,0,0,0,0,0,1.614e-15,-0.09051,0,0,0,0,0,0.10798,0,0,0,0,0.11452,0.16716,0.17472,-0.0065398,0.28939,-0.12101,0,0,3.6056e-17,0.00017277,-0.0024993,0.44663,0,0.11048,0.47822,-0.71091,0,0,-0.048048,-0.0094735,-0.69216,-2.3006,0.10185,-0.021251,-0.6469,0.48737,0,6.917e-16,-0.41068,-0.068907,-0.23636,-0.02154,0,0.031572,0,0.23636,0,0,0,-0.068907,0,0,0,0,0,0,0,0,-1.4196e-15,-0.068907,0,0,0,0,0,-0.25645,0,0,0,0,-0.33907,0.085652,0.033336,0.082615,-0.50053,0.043939,0,0,3.0972e-16,-0.00041032,-0.31882,-0.075419,0,0.062365,-1.9188,0.058963,0,0,-0.0091674,0.0091603,0.082761,0,0.29507,-0.34261,0.67468,-0.19388,0,1.0975e-15,-0.14363,-0.010608,-0.026993,0.081755,0,-0.084122,0,0.026993,0,0,0,-0.010608,0,0,0,0,0,0,0,0,3.4176e-16,-0.010608,0,0,0,0,0,-0.016202,0,0,0,0,-0.02101,0.15742,0.097966,0.004808,0.10005,0.48735,0,0,4.3106e-17,-2.5923e-05,-0.041031,0.30579,0,0.024829,0,1.8594,0,0,-0.026941,0.037194,0.69906,0,-0.16516,1.1194,-0.62213,-0.13665,0,-2.3817e-16,1.0107,0.17879,0.15606,-0.74347,0,0.40634,0,-0.15606,0,0,0,0.17879,0,0,0,0,0,0,0,0,-6.5107e-16,0.17879,0,0,0,0,0,0.1788,0,0,0,0,0.14138,-0.11044,-0.072637,0.037422,-0.052903,0.02565,0,0,-6.9774e-17,0.00028608,0.17259,-0.25548,0,0.0062055,0,0,0,0,0.019975,0.005056,-0.32365,0,-1.4714,0.49624,-0.49862,-0.34876,0,-1.7479e-16,0.10304,0.02038,-0.71921,0.44006,0,0.046797,0,0.71921,0,0,0,0.02038,0,0,0,0,0,0,0,0,1.0302e-15,0.02038,0,0,0,0,0,-0.93995,0,0,0,0,-0.80598,0.26645,-0.2168,-0.13397,0.056684,-0.0035692,0,0,1.7256e-16,-0.0015039,-0.62996,0.19205,0,-0.30999,0,0,0,0,0.059621,-0.011906,-0.29176,0,0,-0.33821,1.2948,-0.15207,0,-4.189e-16,-0.35758,-0.062833,-0.36401,0.16123,0,-0.079817,0,0.36401,0,0,0,-0.062833,0,0,0,0,0,0,0,0,-1.3941e-15,-0.062833,0,0,0,0,0,0.07244,0,0,0,0,0.034803,0.33859,0.08787,0.037638,0.061516,0.015556,0,0,7.1914e-17,0.0001159,-0.03897,0.32689,0,0.11141,0,0,0,0,-0.024164,0.0041387,-0.61819,0,0,0.57922,0,0.43069,0,5.6485e-17,-0.029995,-0.0011351,0.033357,1.1693,0,-0.22086,0,-0.033357,0,0,0,-0.0011351,0,0,0,0,0,0,0,0,1.8289e-15,-0.0011351,0,0,0,0,0,-0.052391,0,0,0,0,-0.047737,-0.050781,-1.148,-0.0046544,0.24357,-0.53051,0,0,2.0731e-16,-8.3826e-05,0.010851,-0.11624,0,-0.063242,0,0,0,0,0.3157,-0.040555,-0.076861,0,0,0.087713,0,-0.073951,0,2.6623e-17,0.0089827,-0.038931,-0.093985,0,0,-0.056828,0,0.093985,0,0,0,-0.038931,0,0,0,0,0,0,0,0,-1.1483e-16,-0.038931,0,0,0,0,0,0.48711,0,0,0,0,0.29817,-0.6107,0,0.18894,-0.041579,0.056227,0,0,-2.1664e-16,0.00077937,0.096772,-0.93465,0,0.39034,0,0,0,0,-4.3772e-16,0.01932,-0.092873,0,0,0.18964,0,-0.055354,0,-4.549e-17,0.077558,0.033693,-0.32607,0,0,0.17784,0,0.32607,0,0,0,0.033693,0,0,0,0,0,0,0,0,1.9684e-15,0.033693,0,0,0,0,0,-0.031166,0,0,0,0,-0.038457,0.014872,0,0.0072909,0.088067,-0.061755,0,0,-9.1848e-17,-4.9866e-05,-0.13269,0,0,0.10152,0,0,0,0,3.9412e-16,-0.0041687,0.36329,0,0,-0.49598,0,0.19194,0,-8.3454e-16,-0.074305,-0.014508,0.85439,0,0,-0.29916,0,-0.85439,0,0,0,-0.014508,0,0,0,0,0,0,0,0,2.4618e-15,-0.014508,0,0,0,0,0,0.091327,0,0,0,0,0.10213,0.051574,0,-0.010805,0.084261,-0.73265,0,0,-1.6462e-16,0.00014612,0.011739,0,0,0.079588,0,0,0,0,1.6077e-16,-0.056077,0.098022,0,0,-0.086283,0,-0.0031733,0,-1.9713e-16,0.0065737,-0.055107,4.2935e-17,0,0,0.094849,0,0,0,0,0,-0.055107,0,0,0,0,0,0,0,0,-1.0548e-15,-0.055107,0,0,0,0,0,0.22537,0,0,0,0,0.20428,0.030222,0,0.021091,0.030002,0,0,0,-8.9518e-17,0.00036059,0.1961,0,0,0.029272,0,0,0,0,1.0709e-16,0.0019556,0.69003,0,0,-0.49393,0,-0.28197,0,-1.1428e-15,0.10981,0.017348,-2.261e-17,0,0,0.40806,0,0,0,0,0,0.017348,0,0,0,0,0,0,0,0,-1.5188e-15,0.017348};
	
	JointMatrix.JointCollision(0, 0, 0, 0, JointMatrix.estParasJoint, A);
	*/
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

struct JointTestParam
{
	double period;
	double amplitude;

};
auto JointTest::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
{
	JointTestParam param;

	for (auto &p : params)
	{
		if (p.first == "period")
			param.period = std::stod(p.second);
		if (p.first == "amplitude")
			param.amplitude = std::stod(p.second);

	}

	target.param = param;

    for(auto &option:target.mot_options) option|=
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

	//读取动力学参数
	auto mat0 = dynamic_cast<aris::dynamic::MatrixVariable*>(&*target.model->variablePool().findByName("estParasJoint"));
	for (int i = 0;i < JointReduceDim + 12;i++)
		JointMatrix.estParasJoint[i] = mat0->data().data()[i];

	auto mat1 = dynamic_cast<aris::dynamic::MatrixVariable*>(&*target.model->variablePool().findByName("CoefParasJoint"));
	for (int i = 0;i < JointReduceDim*JointGroupDim;i++)
		JointMatrix.CoefParasJoint[i] = mat1->data().data()[i];

	auto mat2 = dynamic_cast<aris::dynamic::MatrixVariable*>(&*target.model->variablePool().findByName("CoefParasJointInv"));
	for (int i = 0;i < JointReduceDim*JointGroupDim;i++)
		JointMatrix.CoefParasJointInv[i] = mat2->data().data()[i];

    //auto mat3 = dynamic_cast<aris::dynamic::MatrixVariable*>(&*target.model->variablePool().findByName("LoadParas"));
    for (int i = 0;i < 10;i++)
         JointMatrix.LoadParas[i] = 0;//mat3->data().data()[i];


	double LoadAll[JointGroupDim] = { 0 };
	for (int i = JointGroupDim-10;i < JointGroupDim;i++)
		LoadAll[i] = JointMatrix.LoadParas[i-(JointGroupDim - 10)];

	
	double LoadParasAdd[JointReduceDim] = { 0 };
	s_mm(JointReduceDim, 1, JointGroupDim, JointMatrix.CoefParasJoint, LoadAll, LoadParasAdd);
	for (int i = 0;i < JointReduceDim;i++)
        JointMatrix.estParasJoint[i] = JointMatrix.estParasJoint[i]+ 1*LoadParasAdd[i];

}
auto JointTest::executeRT(PlanTarget &target)->int
{
	auto &param = std::any_cast<JointTestParam&>(target.param);

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
	param.period = 10;


	double CollisionFT[6],q[6],dq[6],ddq[6],ts[6];
	double omega = 0;
	for (int i = 0; i < 6; i++)
	{
		step_pjs[i] = begin_pjs[i] + ampVar *sin(2 * aris::PI / param.period*target.count / 1000);

		target.model->motionPool().at(i).setMp(step_pjs[i]);
		omega = 2 * aris::PI / param.period;
		q[i]= begin_pjs[i] + ampVar * sin(omega * target.count / 1000);
        dq[i] = ampVar * omega * sin(omega*target.count / 1000+aris::PI/2);
        ddq[i] = ampVar * omega* omega * sin(omega*target.count / 1000 + aris::PI);
	}
	double f2c_index[6] = { 9.07327526291993, 9.07327526291993, 17.5690184835913, 39.0310903520972, 66.3992503259041, 107.566785527965 };

	
	// 访问主站 //
	auto controller = target.controller;
	// 打印电流 //
	auto &cout = controller->mout();

	for (int i = 0;i < 6;i++)
	{
		ts[i] = controller->motionAtAbs(i).actualCur() / f2c_index[i];
	}

	double Acv[12] = { 1,1,1,1,1,1,1,1,1,1,1,1 };
    JointMatrix.JointCollision(q, dq, ddq, ts, JointMatrix.estParasJoint, JointMatrix.CoefParasJointInv, JointMatrix.CoefParasJoint, JointMatrix.LoadParas,CollisionFT,Acv);

    if (target.model->solverPool().at(1).kinPos())return -1;
    for (int i = 0;i < 6;i++)
        CollisionFT[i] = CollisionFT[i] - ts[i];
    auto &lout = controller->lout();
    lout << target.count << ",";
    lout << ts[0] << ",";lout << ts[1] << ",";
    lout << ts[2] << ",";lout << ts[3] << ",";
    lout << ts[4] << ",";lout << ts[5] << ",";
    lout << CollisionFT[0] << ",";lout << CollisionFT[1] << ",";
    lout << CollisionFT[2] << ",";lout << CollisionFT[3] << ",";
    lout << CollisionFT[4] << ",";lout <<CollisionFT[5] << ",";

    //lout << q[0] << ",";lout << q[1] << ",";
    //lout << q[2] << ",";lout << q[3] << ",";
    //lout << q[4] << ",";lout << q[5] << ",";

    //lout << dq[0] << ",";lout << dq[1] << ",";
    //lout << dq[2] << ",";lout << dq[3] << ",";
    //lout << dq[4] << ",";lout << dq[5] << ",";

    //lout << ddq[0] << ",";lout << ddq[1] << ",";
    //lout << ddq[2] << ",";lout << ddq[3] << ",";
    //lout << ddq[4] << ",";lout << ddq[5] << ",";

    lout<<endl;
	
	
	if (target.count % 100 == 0)
	{
		//for (int i = 0; i < 6; i++)
		{
            cout << CollisionFT[0] << "***"<< CollisionFT[1] << "***"<< CollisionFT[2] << "***"<< ts[0] << "***"<< ts[1] << "***"<< ts[2] << "***";
			//cout << "vel" << i + 1 << ":" << target.model->motionPool()[i].mv() << "  ";
			//cout << "cur" << i + 1 << ":" << target.model->motionPool()[i].ma() << "  ";
		}
		//   cout << target.count << "  ";
		cout << std::endl;
	}

	

    return 300000 - target.count;
}

JointTest::JointTest(const std::string &name) :Plan(name)
{

	command().loadXmlStr(
		"<Command name=\"JointTest\">"
		"	<GroupParam>"
		"		<Param name=\"period\"default=\"20.0\"/>"
		"		<Param name=\"amplitude\" default=\"0.2\"/>"
		"	</GroupParam>"
		"</Command>");

}


double Acv[12]={0};
struct DragTeachParam
{
	double A1c, A2c, A3c, A4c, A5c, A6c;
	double A1v, A2v, A3v, A4v, A5v, A6v;

};
auto DragTeach::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
{
	DragTeachParam param;
	
	for (auto &p : params)
	{
		if (p.first == "A1c")
			param.A1c = std::stod(p.second);
		if (p.first == "A1v")
			param.A1v = std::stod(p.second);

		if (p.first == "A2c")
			param.A2c = std::stod(p.second);
		if (p.first == "A2v")
			param.A2v = std::stod(p.second);

		if (p.first == "A3c")
			param.A3c = std::stod(p.second);
		if (p.first == "A3v")
			param.A3v = std::stod(p.second);

		if (p.first == "A4c")
			param.A4c = std::stod(p.second);
		if (p.first == "A4v")
			param.A4v = std::stod(p.second);

		if (p.first == "A5c")
			param.A5c = std::stod(p.second);
		if (p.first == "A5v")
			param.A5v = std::stod(p.second);

		if (p.first == "A6c")
			param.A6c = std::stod(p.second);
		if (p.first == "A6v")
			param.A6v = std::stod(p.second);

	}

	target.param = param;

    for(auto &option:target.mot_options) option|=
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
        Plan::NOT_CHECK_VEL_FOLLOWING_ERROR|
        Plan::NOT_CHECK_ENABLE;

	//读取动力学参数
	auto mat0 = dynamic_cast<aris::dynamic::MatrixVariable*>(&*target.model->variablePool().findByName("estParasJoint"));
	for (int i = 0;i < JointReduceDim + 12;i++)
		JointMatrix.estParasJoint[i] = mat0->data().data()[i];

	auto mat1 = dynamic_cast<aris::dynamic::MatrixVariable*>(&*target.model->variablePool().findByName("CoefParasJoint"));
	for (int i = 0;i < JointReduceDim*JointGroupDim;i++)
		JointMatrix.CoefParasJoint[i] = mat1->data().data()[i];

	auto mat2 = dynamic_cast<aris::dynamic::MatrixVariable*>(&*target.model->variablePool().findByName("CoefParasJointInv"));
	for (int i = 0;i < JointReduceDim*JointGroupDim;i++)
		JointMatrix.CoefParasJointInv[i] = mat2->data().data()[i];

    auto mat3 = dynamic_cast<aris::dynamic::MatrixVariable*>(&*target.model->variablePool().findByName("LoadParas"));
    for (int i = 0;i < 10;i++)
         JointMatrix.LoadParas[i] = mat3->data().data()[i];


    double LoadAll[JointGroupDim] = { 0 };
        for (int i = JointGroupDim-10;i < JointGroupDim;i++)
            LoadAll[i] = JointMatrix.LoadParas[i-(JointGroupDim - 10)];


    double LoadParasAdd[JointReduceDim] = { 0 };
    s_mm(JointReduceDim, 1, JointGroupDim, JointMatrix.CoefParasJoint, LoadAll, LoadParasAdd);

    for (int i = 0;i < JointReduceDim;i++)
          JointMatrix.estParasJoint[i] = JointMatrix.estParasJoint[i]+ 1*LoadParasAdd[i];

}
auto DragTeach::executeRT(PlanTarget &target)->int
{
	auto &param = std::any_cast<DragTeachParam&>(target.param);
	
	double f2c_index[6] = { 9.07327526291993, 9.07327526291993, 17.5690184835913, 39.0310903520972, 66.3992503259041, 107.566785527965 };
	static double begin_pjs[6];
	static double step_pjs[6];
	static double perVar = 0;
	static double ampVar = 0;

	// 访问主站 //
	auto controller = target.controller;
	// 打印电流 //
	auto &cout = controller->mout();

	if (target.count == 1)
	{
        for (int i = 0; i < 5; ++i)
		{
			{
				begin_pjs[i] = target.model->motionPool()[i].mp();
                controller->motionPool().at(i).setModeOfOperation(10);	//切换到电流控制
			}
		}
	}


	double ModelTor[6], q[6], dq[6], ddq[6], ts[6];

	Acv[0] = param.A1c;Acv[1] = param.A1v;
	Acv[2] = param.A2c;Acv[3] = param.A2v;
	Acv[4] = param.A3c;Acv[5] = param.A3v;
	Acv[6] = param.A4c;Acv[7] = param.A4v;
	Acv[8] = param.A5c;Acv[9] = param.A5v;
	Acv[10] = param.A6c;Acv[11] = param.A6v;
	
    //for (int i = 0; i < 12; ++i)
        //Acv[i]=0;

	for (int i = 0; i < 6; ++i)
	{
		{
			q[i] = controller->motionAtAbs(i).actualPos();
			dq[i] = controller->motionAtAbs(i).actualVel();
			ddq[i] = 0;
		}
	}
	

	for (int i = 0;i < 6;i++)
	{
		ts[i] = controller->motionAtAbs(i).actualCur() / f2c_index[i];
	}

	JointMatrix.JointDrag(q, dq, ddq, ts, JointMatrix.estParasJoint, JointMatrix.CoefParasJointInv, JointMatrix.CoefParasJoint, JointMatrix.LoadParas, ModelTor,Acv);

    for (int i = 0;i < 5;i++)
	{
		double ft_offset = 0;
		ft_offset = ModelTor[i]*f2c_index[i];
        if(ft_offset>500)
           ft_offset = 500;
        if(ft_offset<-500)
           ft_offset = -500.0;

        controller->motionAtAbs(i).setTargetCur(ft_offset);
	}

    if (target.model->solverPool().at(1).kinPos())return -1;

	auto &lout = controller->lout();
	lout << target.count << ",";
	lout << ts[0] << ",";lout << ts[1] << ",";
	lout << ts[2] << ",";lout << ts[3] << ",";
	lout << ts[4] << ",";lout << ts[5] << ",";
	lout << ModelTor[0] << ",";lout << ModelTor[1] << ",";
	lout << ModelTor[2] << ",";lout << ModelTor[3] << ",";
	lout << ModelTor[4] << ",";lout << ModelTor[5] << ",";

	lout << endl;


	if (target.count % 100 == 0)
	{
		//for (int i = 0; i < 6; i++)
		{
            cout << ModelTor[0] << "***" << ModelTor[1] << "***" << ModelTor[2] << "***" << ModelTor[3] << "***" << ModelTor[4] << "***" << ModelTor[5] << "***";
            //cout << "vel" << i + 1 << ":" << target.model->motionPool()[i].mv() << "  ";
			//cout << "cur" << i + 1 << ":" << target.model->motionPool()[i].ma() << "  ";
		}
		//   cout << target.count << "  ";
		cout << std::endl;
	}



    return 300000 - target.count;
}



DragTeach::DragTeach(const std::string &name) :Plan(name)
{

	command().loadXmlStr(
		"<Command name=\"DragTeach\">"
		"	<GroupParam>"
		"		<Param name=\"A1c\"default=\"0.5\"/>"
		"		<Param name=\"A1v\"default=\"0.2\"/>"
		"		<Param name=\"A2c\"default=\"0.5\"/>"
		"		<Param name=\"A2v\"default=\"0.2\"/>"
		"		<Param name=\"A3c\"default=\"0.5\"/>"
		"		<Param name=\"A3v\"default=\"0.2\"/>"
		"		<Param name=\"A4c\"default=\"0.5\"/>"
		"		<Param name=\"A4v\"default=\"0.2\"/>"
		"		<Param name=\"A5c\"default=\"0.5\"/>"
		"		<Param name=\"A5v\"default=\"0.2\"/>"
		"		<Param name=\"A6c\"default=\"0.5\"/>"
		"		<Param name=\"A6v\"default=\"0.2\"/>"
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

    for(auto &option:target.mot_options) option|=
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
	static double begin_pjs[RobotAxis];
	static double step_pjs[RobotAxis];
	static double perVar = 0;
    static int CollectNum=1;
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






    if (target.model->solverPool().at(1).kinPos())return -1;

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
          cout << CollectNum << "  "<< step_pjs[2]<<" "<<flag[2]<<" "<<t_count[2]<<" "<<target.count - (CountOffsetNeg[2] - 1)<<" ";
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
			TorqueList[6 * (CollectNum - 1) + i] = 0;//controller->motionAtAbs(i).actualCur() / f2c_index[i];
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
	double StatisError[3] = {0,0,0};
	//  auto controller = target.controller;
	 // auto &lout = controller->lout();
    std::cout << "param.InitEst" << std::endl;
	
	//读取YYbase 参数
	auto mat0 = dynamic_cast<aris::dynamic::MatrixVariable*>(&*target.model->variablePool().findByName("CoefParasLoad"));
	for (int i = 0;i < 13 * 40;i++)
		JointMatrix.CoefParasLoad[i] = mat0->data().data()[i];

	auto mat1 = dynamic_cast<aris::dynamic::MatrixVariable*>(&*target.model->variablePool().findByName("CoefParasLoadInv"));
	for (int i = 0;i < 13 * 40;i++)
		JointMatrix.CoefParasLoadInv[i] = mat1->data().data()[i];
	//double data[10]{ 0.0 };
	//aris::core::Matrix mat(10,1,data);
	//aris::core::Matrix mat2 = { { 1.0,2.0,3.0 }, { 1.0,2.0,3.0 } };
	//target.model->variablePool().add<aris::dynamic::MatrixVariable>("p0", mat);
	
	//auto mat = dynamic_cast<aris::dynamic::MatrixVariable*>(&*target.model->variablePool().findByName("p0"));


	if (param.amplitude > 0)
	{
        JointMatrix.LoadRLS(AngleList, TorqueList, JointMatrix.CoefParasLoad,JointMatrix.CoefParasLoadInv,JointMatrix.estParasL0, StatisError);
		for (int i = 0;i < LoadReduceParas + 6;i++)
			cout << JointMatrix.estParasL0[i] << ",";

		aris::core::Matrix mat0(1,LoadReduceParas + 6, JointMatrix.estParasL0);
		if (target.model->variablePool().findByName("estParasL0") !=
			target.model->variablePool().end())
		{
			dynamic_cast<aris::dynamic::MatrixVariable*>(
				&*target.model->variablePool().findByName("estParasL0"))->data() = mat0;
		}
		else
		{
			target.model->variablePool().add<aris::dynamic::MatrixVariable>("estParasL0", mat0);
		}




		cout << "collect0" <<endl;
	}
	else
	{

		auto mat0 = dynamic_cast<aris::dynamic::MatrixVariable*>(&*target.model->variablePool().findByName("estParasL0"));
		for (int i = 0;i < LoadReduceParas + 6;i++)
			JointMatrix.estParasL0[i] = mat0->data().data()[i];

        JointMatrix.LoadRLS(AngleList, TorqueList, JointMatrix.CoefParasLoad, JointMatrix.CoefParasLoadInv,JointMatrix.estParasL, StatisError);


		for (int i = 0;i < LoadReduceParas;i++)
			dEst[i] = JointMatrix.estParasL[i] - JointMatrix.estParasL0[i];



        JointMatrix.LoadParasExt(dEst,JointMatrix.CoefParasLoad, JointMatrix.CoefParasLoadInv,JointMatrix.LoadParas);
		for (int i = 0;i < 10;i++)
			cout << JointMatrix.LoadParas[i] << ",";

		aris::core::Matrix mat2(1,10, JointMatrix.LoadParas);
		if (target.model->variablePool().findByName("LoadParas") !=
			target.model->variablePool().end())
		{
			dynamic_cast<aris::dynamic::MatrixVariable*>(
				&*target.model->variablePool().findByName("LoadParas"))->data() = mat2;
		}
		else
		{
			target.model->variablePool().add<aris::dynamic::MatrixVariable>("LoadParas", mat2);
		}
		
    }



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

std::vector<double> AngList_vec(6 * 1250);
auto AngList = AngList_vec.data();
std::vector<double> VelList_vec(6 * 1250);
auto VelList = VelList_vec.data();
std::vector<double> AccList_vec(6 * 1250);
auto AccList = AccList_vec.data();

auto SaveYYbase::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
{
	double x[67] = {0,0.27675977, -0.00008492, 0.45121602, -0.05622749, 0.05870555, 0.32202060, 0.13967663, 0.06755620, 0.41973365, 0.12796641, -0.45479368, 0.06346175, -0.03418769, -1.43096503, 0.19009482, 0.10024365, 0.02245434, -1.11449636, -0.30552461, 0.17608308, -0.20557476, -0.18852254, -0.04907027, -0.05084139, -0.34651352, 0.60754337, 0.45868045, 0.32841629, 0.18534060, -0.16925049, 0.17826802, 0.32594942, -0.47694434, 0.44657118, 0.05735728, 1.23755867, 0.14668329, 0.79646119, 0.24766397, 0.12742196, -0.02780189, -0.29925711, 0.07980669, -0.32086920, 0.66890123, -0.23126394, 0.02150397, 0.01039145, 0.04520572, 0.08193191, 0.19993891, 0.14874881, 0.77013426, 0.10203477, -0.12763027, 0.58259397, 0.80006153, -0.10343947, -0.26281997, 0.06988126, -0.20153010, 0.19942194, 0.05188115, -0.24163165, 0.00498271, 0.00960368 };
	int fourierNum = 5;
	int	nParas = 2 * fourierNum + 1;
	int	linkNum = 3;
	double	Tf = 10;
	double omega = 2 * aris::PI / Tf;
	double dt = 0.008, timeSin = 0;
	int TestNum = Tf / dt;
	double q[4][6] = { 0 }, dq[4][6] = { 0 }, ddq[4][6] = { 0 };
	double qF[4] = { 0 }, dqF[4] = { 0 }, ddqF[4] = { 0 };
    int l = 0;
	double a, b;
	for (int i = 0;i < TestNum;i++)
	{
		timeSin = dt * i;
		for (int j = 1;j < linkNum + 1;j++)
		{
			l = 1;
			q[j][1] = x[1 + (j - 1)*nParas] / (omega*l)*sin(omega*l*timeSin) - x[2 + (j - 1)*nParas] / (omega*l)*cos(omega*l*timeSin) + x[3 + (j - 1)*nParas];
            dq[j][1] = x[1 + (j - 1)*nParas]*cos(omega*l*timeSin) + x[2 + (j - 1)*nParas]*sin(omega*l*timeSin);
			ddq[j][1] = -x[1 + (j - 1)*nParas]*omega*l*sin(omega*l*timeSin) + x[2 + (j - 1)*nParas]*omega*l*cos(omega*l*timeSin);
		}

		for (int j = 1;j < linkNum + 1;j++)
			for (int ii = 2;ii < fourierNum + 1;ii++)
			{
				l = ii;
				q[j][ii] = x[2 + (ii - 2) * 2 + 2 + (j - 1)*nParas] / (omega*l)*sin(omega*l*timeSin) - x[2 + (ii - 2) * 2 + 3 + (j - 1)*nParas] / (omega*l)*cos(omega*l*timeSin);
			    dq[j][ii] = x[2 + (ii - 2) * 2 + 2 + (j - 1)*nParas]*cos(omega*l*timeSin) + x[2 + (ii - 2) * 2 + 3 + (j - 1)*nParas]*sin(omega*l*timeSin);
				ddq[j][ii] = -x[2 + (ii - 2) * 2 + 2 + (j - 1)*nParas]*omega*l*sin(omega*l*timeSin) + x[2 + (ii - 2) * 2 + 3 + (j - 1)*nParas]*omega*l*cos(omega*l*timeSin);

			}

		for (int j = 1;j < linkNum + 1;j++)
		{
			qF[j] = q[j][1];
			dqF[j] = dq[j][1];
			ddqF[j] = ddq[j][1];
		}


		for (int j = 1;j < linkNum + 1;j++)
			for (int ii = 2;ii < fourierNum + 1;ii++)
			{
				qF[j] = qF[j] + q[j][ii];
				dqF[j] = dqF[j] + dq[j][ii];
				ddqF[j] = ddqF[j] + ddq[j][ii];
			}
	
		AngList[6 * i + 0] = 0; VelList[6 * i + 0] = 0; AccList[6 * i + 0] = 0;
		AngList[6 * i + 1] = 0; VelList[6 * i + 1] = 0; AccList[6 * i + 1] = 0;
		AngList[6 * i + 2] = qF[1]; VelList[6 * i + 2] = dqF[1]; AccList[6 * i + 2] = ddqF[1];
		AngList[6 * i + 3] = 0; VelList[6 * i + 3] = 0; AccList[6 * i + 3] = 0;
		AngList[6 * i + 4] = qF[2]; VelList[6 * i + 4] = dqF[2]; AccList[6 * i + 4] = ddqF[2];
		AngList[6 * i + 5] = qF[3]; VelList[6 * i + 5] = dqF[3]; AccList[6 * i + 5] = ddqF[3];
	}



	JointMatrix.YYbase(AngList, VelList, AccList,JointMatrix.Load2Joint, JointMatrix.CoefParasLoad, JointMatrix.CoefParasLoadInv, TestNum);

	aris::core::Matrix mat0(1, 13*40, JointMatrix.CoefParasLoad);
	if (target.model->variablePool().findByName("CoefParasLoad") !=
		target.model->variablePool().end())
	{
		dynamic_cast<aris::dynamic::MatrixVariable*>(
			&*target.model->variablePool().findByName("CoefParasLoad"))->data() = mat0;
	}
	else
	{
		target.model->variablePool().add<aris::dynamic::MatrixVariable>("CoefParasLoad", mat0);
	}

	aris::core::Matrix mat1(1, 13*40, JointMatrix.CoefParasLoadInv);
	if (target.model->variablePool().findByName("CoefParasLoadInv") !=
		target.model->variablePool().end())
	{
		dynamic_cast<aris::dynamic::MatrixVariable*>(
			&*target.model->variablePool().findByName("CoefParasLoadInv"))->data() = mat1;
	}
	else
	{
		target.model->variablePool().add<aris::dynamic::MatrixVariable>("CoefParasLoadInv", mat1);
	}




	target.option |= NOT_RUN_COLLECT_FUNCTION;
	target.option |= NOT_RUN_EXECUTE_FUNCTION;
}

SaveYYbase::SaveYYbase(const std::string &name) :Plan(name)
{
	command().loadXmlStr(
		"<Command name=\"svYYbase\">"
		"</Command>");
}






struct SaveFileParam
{
	string gk_path;
};

auto SaveFile::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
{
    auto xmlpath = std::filesystem::absolute(".");//获取当前工程所在的路径
    const std::string xmlfile = "kaanh.xml";
    xmlpath = xmlpath / xmlfile;
	auto&cs = aris::server::ControlServer::instance();
	SaveFileParam p;
	p.gk_path = params.at("gk_path");

    cs.saveXmlFile(xmlpath.string().c_str());
	//target.server->stop();
	//target.server->saveXmlFile("C:/Users/qianch_kaanh_cn/Desktop/build_qianch/rokae.xml");		
	//doc.SaveFile("C:/Users/qianch_kaanh_cn/Desktop/build_qianch/rokae.xml");
	target.option |= NOT_RUN_COLLECT_FUNCTION;
	target.option |= NOT_RUN_EXECUTE_FUNCTION;
}

SaveFile::SaveFile(const std::string &name) :Plan(name)
{
	command().loadXmlStr(
		"<Command name=\"svFi\">"
		"	<GroupParam>"
		"	    <Param name=\"gk_path\" default=\"C:/Users/gk/Desktop/build_kaanh_gk/kaanh.xml\"/>"
		"	</GroupParam>"
		"</Command>");
}



