#include "jointfc.h"
#include <math.h>
#include <algorithm>
#include"robotconfig.h"
#include"jointdynamics.h"

//using namespace std;
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
std::vector<std::vector<double>> POSRLS(13);
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
		Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
		Plan::NOT_CHECK_POS_FOLLOWING_ERROR |
		//#endif
		Plan::NOT_CHECK_VEL_MIN |
		Plan::NOT_CHECK_VEL_MAX |
		Plan::NOT_CHECK_VEL_CONTINUOUS |
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

	if (CollectNum < SampleNum)
	{
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
	}
	else
	{
		for (int i = 0;i < 6;i++)
		{

			if (flag[i])
			{
				if (vArc[i] > 0.0001*1e-4)
				{
					vArc[i] = vArc[i] - 0.001*10e-3;
					step_pjs[i] = step_pjs[i] + vArc[i];
				}

			}
			if (flag[i] == false)
			{

				if (vArc[i] > 0.0001*1e-4)
				{
					vArc[i] = vArc[i] - 0.001*1e-3;
					step_pjs[i] = step_pjs[i] - vArc[i];
				}


			}
				target.model->motionPool().at(i).setMp(step_pjs[i]);
		}
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

    if (target.count % 8 == 0 && CollectNum < SampleNum)
	{
		for (int i = 0; i < 6; i++)
		{
			//AngleList[RobotAxis * (target.count - 1) + i] = controller->motionAtAbs(i).actualPos();
			//TorqueList[RobotAxis * (target.count - 1) + i] = controller->motionAtAbs(i).actualCur();

            //AngleList[6 * (target.count - 1) + i] = POSRLS[i][target.count - 1];
            //TorqueList[6 * (target.count - 1) + i] = POSRLS[i + 6][target.count - 1] / f2c_index[i];
#ifdef UNIX
            AngleList[6 * (CollectNum - 1) + i] = controller->motionAtAbs(i).actualPos();
            TorqueList[6 * (CollectNum - 1) + i] = controller->motionAtAbs(i).actualCur() / f2c_index[i];
#endif
            //TorqueList[6 * (CollectNum - 1) + i] = 2;//controller->motionAtAbs(i).actualTor() * f2f_index[i];
		}

		lout << target.count << ",";lout << vArc[2] << ",";
		lout << AngleList[RobotAxis * (CollectNum - 1) + 0] << ",";lout << AngleList[RobotAxis * (CollectNum - 1) + 1] << ",";
		lout << AngleList[RobotAxis * (CollectNum - 1) + 2] << ",";lout << AngleList[RobotAxis * (CollectNum - 1) + 3] << ",";
		lout << AngleList[RobotAxis * (CollectNum - 1) + 4] << ",";lout << AngleList[RobotAxis * (CollectNum - 1) + 5] << ",";
		lout << TorqueList[RobotAxis * (CollectNum - 1) + 0] << ",";lout << TorqueList[RobotAxis * (CollectNum - 1) + 1] << ",";
		lout << TorqueList[RobotAxis * (CollectNum - 1) + 2] << ",";lout << TorqueList[RobotAxis * (CollectNum - 1) + 3] << ",";
		lout << TorqueList[RobotAxis * (CollectNum - 1) + 4] << ",";lout << TorqueList[RobotAxis * (CollectNum - 1) + 5] << ",";

		lout << std::endl;
		CollectNum = CollectNum + 1;
	}

	if (target.count % 8 == 0 && CollectNum > SampleNum - 1)
	{
		CollectNum = CollectNum + 1;

		lout << target.count << ",";lout << vArc[2] << ",";
		lout << AngleList[RobotAxis * (CollectNum - 1) + 0] << ",";lout << AngleList[RobotAxis * (CollectNum - 1) + 1] << ",";
		lout << AngleList[RobotAxis * (CollectNum - 1) + 2] << ",";lout << AngleList[RobotAxis * (CollectNum - 1) + 3] << ",";
		lout << AngleList[RobotAxis * (CollectNum - 1) + 4] << ",";lout << AngleList[RobotAxis * (CollectNum - 1) + 5] << ",";
		lout << TorqueList[RobotAxis * (CollectNum - 1) + 0] << ",";lout << TorqueList[RobotAxis * (CollectNum - 1) + 1] << ",";
		lout << TorqueList[RobotAxis * (CollectNum - 1) + 2] << ",";lout << TorqueList[RobotAxis * (CollectNum - 1) + 3] << ",";
		lout << TorqueList[RobotAxis * (CollectNum - 1) + 4] << ",";lout << TorqueList[RobotAxis * (CollectNum - 1) + 5] << ",";

		lout << std::endl;
	}


	return (SampleNum+150) - CollectNum;
}


auto JointDyna::collectNrt(aris::plan::PlanTarget &target)->void
{
	
	double StatisError[6] = { 0,0,0,0,0,0 };
	auto controller = target.controller;
	 // auto &lout = controller->lout();
	auto &cout = controller->mout();
	cout << "collect" << std::endl;

   /*RLS Kai
	auto mat1 = dynamic_cast<aris::dynamic::MatrixVariable*>(&*target.model->variablePool().findByName("CoefParasJoint"));
	for (int i = 0;i < JointReduceDim*JointGroupDim;i++)
		JointMatrix.CoefParasJoint[i] = mat1->data().data()[i];

	auto mat2 = dynamic_cast<aris::dynamic::MatrixVariable*>(&*target.model->variablePool().findByName("CoefParasJointInv"));
	for (int i = 0;i < JointReduceDim*JointGroupDim;i++)
		JointMatrix.CoefParasJointInv[i] = mat2->data().data()[i];

	
    JointMatrix.RLStemp(AngleList, TorqueList, JointMatrix.estParasJoint, JointMatrix.CoefParasJoint, JointMatrix.CoefParasJointInv, StatisError);
	//cout<<"collect"<<std::endl;
	for (int i = 0;i < JointReduceDim+12;i++)
		cout << JointMatrix.estParasJoint[i] << ",";

	aris::core::Matrix mat0(1,JointReduceDim + 12, JointMatrix.estParasJoint);
	if (target.model->variablePool().findByName(".") !=
		target.model->variablePool().end())
	{
		dynamic_cast<aris::dynamic::MatrixVariable*>(
			&*target.model->variablePool().findByName("estParasJoint"))->data() = mat0;
	}
	else
	{
		target.model->variablePool().add<aris::dynamic::MatrixVariable>("estParasJoint", mat0);
	}*/


	///*RLS Yang
	JointMatrix.RLSYang(AngleList, TorqueList, JointMatrix.estParasJointYang, StatisError);
	//cout<<"collect"<<std::endl;
	for (int i = 0;i < JointGroupDim + 12;i++)
		cout << JointMatrix.estParasJointYang[i] << ",";

	cout << std::endl;
	cout << "*****************************Statictic Model Error*****************************************" << std::endl;
	for (int i = 0;i < RobotAxis;i++)
		cout << StatisError[i] << std::endl;


	std::vector<std::pair<std::string, std::any>> out_param;
	out_param.push_back(std::make_pair<std::string, std::any>("link_param", JointMatrix.estParasJointYang));
	out_param.push_back(std::make_pair<std::string, std::any>("torque_error", StatisError));
	target.ret = out_param;


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

struct JointDynaSaveParam
{
	double period;
	double amplitude;

};
auto JointDynaSave::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
{
	JointDynaSaveParam param;

	for (auto &p : params)
	{
		if (p.first == "period")
			param.period = std::stod(p.second);
		if (p.first == "amplitude")
			param.amplitude = std::stod(p.second);

	}

	target.param = param;

	aris::core::Matrix mat0(1, JointGroupDim + 12, JointMatrix.estParasJointYang);
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

}
JointDynaSave::JointDynaSave(const std::string &name) :Plan(name)
{

	command().loadXmlStr(
		"<Command name=\"JointDynaSave\">"
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
		Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
		Plan::NOT_CHECK_POS_FOLLOWING_ERROR |
		//#endif
		Plan::NOT_CHECK_VEL_MIN |
		Plan::NOT_CHECK_VEL_MAX |
		Plan::NOT_CHECK_VEL_CONTINUOUS |
		Plan::NOT_CHECK_VEL_FOLLOWING_ERROR;

	//读取动力学参数
	/*KAI
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
         JointMatrix.LoadParas[i] = 0*mat3->data().data()[i];
    for (int i = 0;i < 6;i++)
         JointMatrix.LoadParas[i] = 0;


	double LoadAll[JointGroupDim] = { 0 };
	for (int i = JointGroupDim-10;i < JointGroupDim;i++)
		LoadAll[i] = JointMatrix.LoadParas[i-(JointGroupDim - 10)];

	
	double LoadParasAdd[JointReduceDim] = { 0 };
	s_mm(JointReduceDim, 1, JointGroupDim, JointMatrix.CoefParasJoint, LoadAll, LoadParasAdd);
	for (int i = 0;i < JointReduceDim;i++)
        JointMatrix.estParasJoint[i] = JointMatrix.estParasJoint[i]+ 1*LoadParasAdd[i];
		*/

	///*Yang
	auto mat0 = dynamic_cast<aris::dynamic::MatrixVariable*>(&*target.model->variablePool().findByName("estParasJoint"));
	for (int i = 0;i < JointGroupDim + 12;i++)
		JointMatrix.estParasJoint[i] = mat0->data().data()[i];

	auto mat3 = dynamic_cast<aris::dynamic::MatrixVariable*>(&*target.model->variablePool().findByName("LoadParas"));
	for (int i = 0;i < 10;i++)
         JointMatrix.LoadParas[i] = 1*mat3->data().data()[i];
	

	//for (int i = 50;i < JointGroupDim;i++)
		//JointMatrix.estParasJoint[i] = JointMatrix.estParasJoint[i]+ JointMatrix.LoadParas[i-50];
		

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
    //JointMatrix.JointCollision(q, dq, ddq, ts, JointMatrix.estParasJoint, JointMatrix.CoefParasJointInv, JointMatrix.CoefParasJoint, JointMatrix.LoadParas,CollisionFT,Acv);

	JointMatrix.JointCollisionYang(q, dq, ddq, ts, JointMatrix.estParasJoint, JointMatrix.LoadParas, CollisionFT, Acv);

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

    lout<<std::endl;
	
	
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
		Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
		Plan::NOT_CHECK_POS_FOLLOWING_ERROR |
		//#endif
		Plan::NOT_CHECK_VEL_MIN |
		Plan::NOT_CHECK_VEL_MAX |
		Plan::NOT_CHECK_VEL_CONTINUOUS |
        Plan::NOT_CHECK_VEL_FOLLOWING_ERROR|
        Plan::NOT_CHECK_ENABLE;

	//读取动力学参数
	/*KAI
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

    for (int i = 0;i < 6;i++)
         JointMatrix.LoadParas[i] = 0;

    double LoadAll[JointGroupDim] = { 0 };
        for (int i = JointGroupDim-10;i < JointGroupDim;i++)
            LoadAll[i] = JointMatrix.LoadParas[i-(JointGroupDim - 10)];


    double LoadParasAdd[JointReduceDim] = { 0 };
    s_mm(JointReduceDim, 1, JointGroupDim, JointMatrix.CoefParasJoint, LoadAll, LoadParasAdd);

    for (int i = 0;i < JointReduceDim;i++)
          JointMatrix.estParasJoint[i] = JointMatrix.estParasJoint[i]+ 1*LoadParasAdd[i];
		  */

		  ///*Yang
	auto mat0 = dynamic_cast<aris::dynamic::MatrixVariable*>(&*target.model->variablePool().findByName("estParasJoint"));
	for (int i = 0;i < JointGroupDim + 12;i++)
		JointMatrix.estParasJoint[i] = mat0->data().data()[i];

	auto mat3 = dynamic_cast<aris::dynamic::MatrixVariable*>(&*target.model->variablePool().findByName("LoadParas"));
	for (int i = 0;i < 10;i++)
        JointMatrix.LoadParas[i] = 1 * mat3->data().data()[i];
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
        for (int i = 0; i < 6; ++i)
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

	//JointMatrix.JointDrag(q, dq, ddq, ts, JointMatrix.estParasJoint, JointMatrix.CoefParasJointInv, JointMatrix.CoefParasJoint, JointMatrix.LoadParas, ModelTor,Acv);
	JointMatrix.JointDragYang(q, dq, ddq, ts, JointMatrix.estParasJoint, JointMatrix.LoadParas, ModelTor, Acv);


    for (int i = 0;i < 5;i++)
	{
		double ft_offset = 0;
		ft_offset = ModelTor[i]*f2c_index[i];
        if(ft_offset>500)
           ft_offset = 500;
        if(ft_offset<-500)
           ft_offset = -500.0;

        controller->motionAtAbs(i).setTargetToq(ft_offset);
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

	lout << std::endl;


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

	std::fill(target.mot_options.begin(), target.mot_options.end(),
		//用于使用模型轨迹驱动电机//
		Plan::USE_TARGET_POS);
/*
    for(auto &option:target.mot_options) option|=
		Plan::USE_TARGET_POS |
		//#ifdef WIN32
		Plan::NOT_CHECK_POS_MIN |
		Plan::NOT_CHECK_POS_MAX |
		Plan::NOT_CHECK_POS_CONTINUOUS |
		Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
		Plan::NOT_CHECK_POS_FOLLOWING_ERROR |
		//#endif
		Plan::NOT_CHECK_VEL_MIN |
		Plan::NOT_CHECK_VEL_MAX |
		Plan::NOT_CHECK_VEL_CONTINUOUS |
		Plan::NOT_CHECK_VEL_FOLLOWING_ERROR;
*/

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
    double PosLimit[6] = { 1,1,0.2,1,1.5,2.5 };
    double NegLimit[6] = { -1,-1,-0.2,-1,-1.5,-2.5 };
	double dTheta = 0.0001;
	static double pArc[6], vArc[6], aArc[6], vArcMax[6] = { 0.15,0.15,0.15,0.15,0.15,0.15 };
	static aris::Size t_count[6] = { 0 };
	
	static int CountOffsetPos[6] = { 1,1,1,1,1,1 }, CountOffsetNeg[6] = { 1,1,1,1,1,1};


	if (CollectNum < SampleNum)
		for (int i = 0;i < 6;i++)
	{
		
		if (flag[i])
		{
			if (step_pjs[i] < PosLimit[i])
			{
				aris::plan::moveAbsolute(target.count - CountOffsetNeg[i] + 1, 0, PosLimit[i] - begin_pjs[i], vArcMax[i] / 1000, 0.05 / 1000 / 1000, 0.05 / 1000 / 1000, pArc[i], vArc[i], aArc[i], t_count[i]);

				step_pjs[i] = step_pjs[i] + vArc[i];
			}
				//cout << vArc << "  ";
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
	else
		for (int i = 0;i < 6;i++)
		{

			if (flag[i])
			{
				if (vArc[i] > 0.0001*1e-4)
				{
					vArc[i] = vArc[i] - 0.001*1e-4;
				    step_pjs[i] = step_pjs[i] + vArc[i];
				}

			}
			if (flag[i] == false)
			{

				if (vArc[i] > 0.0001*1e-4)
				{
					vArc[i] = vArc[i] - 0.001*1e-4;
					step_pjs[i] = step_pjs[i] - vArc[i];
			     }


			}
			if (i == 2 || i == 4 || i == 5)
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
          cout << CollectNum << "  "<< vArc[2] <<" "<<flag[2]<<" "<<t_count[2]<<" "<<target.count - (CountOffsetNeg[2] - 1)<<" ";
		cout << std::endl;
	}

	auto &lout = controller->lout();

	double f2c_index[6] = { 9.07327526291993, 9.07327526291993, 17.5690184835913, 39.0310903520972, 66.3992503259041, 107.566785527965 };

	

    if(target.count%8==0&& CollectNum< SampleNum)
    {
        for (int i = 0; i < 6; i++)
        {
            //AngleList[RobotAxis * (target.count - 1) + i] = controller->motionAtAbs(i).actualPos();
            //TorqueList[RobotAxis * (target.count - 1) + i] = controller->motionAtAbs(i).actualCur();

            //AngleList[6 * (target.count - 1) + i] = POSRLS[i][target.count - 1];
            //TorqueList[6 * (target.count - 1) + i] = POSRLS[i + 6][target.count - 1];
#ifdef UNIX
            AngleList[6 * (CollectNum - 1) + i] = controller->motionAtAbs(i).actualPos();
            TorqueList[6 * (CollectNum - 1) + i] = controller->motionAtAbs(i).actualCur() / f2c_index[i];
#endif
        }
	
        lout << target.count << ",";lout << vArc[2] << ",";
        lout << AngleList[RobotAxis * (CollectNum - 1) + 0] << ",";lout << AngleList[RobotAxis * (CollectNum - 1) + 1] << ",";
        lout << AngleList[RobotAxis * (CollectNum - 1) + 2] << ",";lout << AngleList[RobotAxis * (CollectNum - 1) + 3] << ",";
        lout << AngleList[RobotAxis * (CollectNum - 1) + 4] << ",";lout << AngleList[RobotAxis * (CollectNum - 1) + 5] << ",";
        lout << TorqueList[RobotAxis * (CollectNum - 1) + 0] << ",";lout << TorqueList[RobotAxis * (CollectNum - 1) + 1] << ",";
        lout << TorqueList[RobotAxis * (CollectNum - 1) + 2] << ",";lout << TorqueList[RobotAxis * (CollectNum - 1) + 3] << ",";
        lout << TorqueList[RobotAxis * (CollectNum - 1) + 4] << ",";lout << TorqueList[RobotAxis * (CollectNum - 1) + 5] << ",";

        lout << std::endl;
        CollectNum=CollectNum+1;
    }
	
	if (target.count % 8 == 0 && CollectNum > SampleNum-1)
	{
		CollectNum = CollectNum + 1;

		lout << target.count << ",";lout << vArc[2] << ",";
		lout << AngleList[RobotAxis * (CollectNum - 1) + 0] << ",";lout << AngleList[RobotAxis * (CollectNum - 1) + 1] << ",";
		lout << AngleList[RobotAxis * (CollectNum - 1) + 2] << ",";lout << AngleList[RobotAxis * (CollectNum - 1) + 3] << ",";
		lout << AngleList[RobotAxis * (CollectNum - 1) + 4] << ",";lout << AngleList[RobotAxis * (CollectNum - 1) + 5] << ",";
		lout << TorqueList[RobotAxis * (CollectNum - 1) + 0] << ",";lout << TorqueList[RobotAxis * (CollectNum - 1) + 1] << ",";
		lout << TorqueList[RobotAxis * (CollectNum - 1) + 2] << ",";lout << TorqueList[RobotAxis * (CollectNum - 1) + 3] << ",";
		lout << TorqueList[RobotAxis * (CollectNum - 1) + 4] << ",";lout << TorqueList[RobotAxis * (CollectNum - 1) + 5] << ",";

		lout << std::endl;
	}


    return (150+SampleNum) - CollectNum;
}


auto LoadDyna::collectNrt(aris::plan::PlanTarget &target)->void
{
	auto &param = std::any_cast<LoadDynaParam&>(target.param);
    double dEst[LoadReduceParas] = { 0 };
	double dCoef[LoadReduceParas*10] = { 0 };
	double StatisError[3] = {0,0,0};
    auto controller = target.controller;
	auto &cout = controller->mout();
	 // auto &lout = controller->lout();
    cout << "param.InitEst" << std::endl;
	
	/*LoadRLSKai
	auto mat0 = dynamic_cast<aris::dynamic::MatrixVariable*>(&*target.model->variablePool().findByName("CoefParasLoad"));
	for (int i = 0;i < LoadTotalParas * LoadReduceParas;i++)
		JointMatrix.CoefParasLoad[i] = mat0->data().data()[i];

	auto mat1 = dynamic_cast<aris::dynamic::MatrixVariable*>(&*target.model->variablePool().findByName("CoefParasLoadInv"));
	for (int i = 0;i < LoadTotalParas * LoadReduceParas;i++)
		JointMatrix.CoefParasLoadInv[i] = mat1->data().data()[i];

	if (param.amplitude > 0)
	{
        JointMatrix.LoadRLStemp(AngleList, TorqueList, JointMatrix.CoefParasLoad,JointMatrix.CoefParasLoadInv,JointMatrix.estParasL0, StatisError);
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




		cout << "collect0" <<std::endl;
	}
	else
	{

		auto mat0 = dynamic_cast<aris::dynamic::MatrixVariable*>(&*target.model->variablePool().findByName("estParasL0"));
		for (int i = 0;i < LoadReduceParas + 6;i++)
			JointMatrix.estParasL0[i] = mat0->data().data()[i];

        JointMatrix.LoadRLStemp(AngleList, TorqueList, JointMatrix.CoefParasLoad, JointMatrix.CoefParasLoadInv,JointMatrix.estParasL, StatisError);

        for (int i = 0;i < LoadReduceParas + 6;i++)
            cout << JointMatrix.estParasL[i] << ",";
cout << std::endl;
cout << std::endl;
		for (int i = 0;i < LoadReduceParas;i++)
			dEst[i] = JointMatrix.estParasL[i] - JointMatrix.estParasL0[i];



        JointMatrix.LoadParasExt(dEst,JointMatrix.CoefParasLoad, JointMatrix.CoefParasLoadInv,JointMatrix.LoadParas);
		for (int i = 0;i < 10;i++)
			cout << JointMatrix.LoadParas[i] << ",";
        cout << std::endl;
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
		
    }*/

	//*LoadRLSYang
	
	if (param.amplitude > 0)
	{
		JointMatrix.LoadRLSYang(AngleList, TorqueList, JointMatrix.estParasL0Yang, StatisError);
		for (int i = 0;i < LoadTotalParas + 6;i++)
			cout << JointMatrix.estParasL0Yang[i] << ",";

	    cout << "*****************************Statictic Model Error*****************************************" << std::endl;
		for (int i = 0;i < 3;i++)
			cout << StatisError[i] << std::endl;

		std::vector<std::pair<std::string, std::any>> out_param;
		out_param.push_back(std::make_pair<std::string, std::any>("estParasL0", JointMatrix.estParasL0Yang));
		out_param.push_back(std::make_pair<std::string, std::any>("torque_error", StatisError));
		target.ret = out_param;
		
	}
	else
	{
		auto mat0 = dynamic_cast<aris::dynamic::MatrixVariable*>(&*target.model->variablePool().findByName("estParasL0"));
		for (int i = 0;i < LoadTotalParas + 6;i++)
			JointMatrix.estParasL0Yang[i] = mat0->data().data()[i];

		//JointMatrix.LoadRLStemp(AngleList, TorqueList, JointMatrix.CoefParasLoad, JointMatrix.CoefParasLoadInv,JointMatrix.estParasL, StatisError);

		JointMatrix.LoadParasExtYang(AngleList, TorqueList, JointMatrix.estParasL0Yang,JointMatrix.LoadParas, StatisError);
		for (int i = 0;i < 10;i++)
			cout << JointMatrix.LoadParas[i] << ",";
	
		std::cout << "*****************************Statictic Model Error*****************************************" << std::endl;
		for (int i = 0;i < 3;i++)
			cout << StatisError[i] << std::endl;

		std::vector<std::pair<std::string, std::any>> out_param;
		out_param.push_back(std::make_pair<std::string, std::any>("load_params", JointMatrix.LoadParas));
		out_param.push_back(std::make_pair<std::string, std::any>("torque_error", StatisError));
		target.ret = out_param;

	}


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

struct LoadDynaSave0Param
{
	double period;
	double amplitude;

};
auto LoadDynaSave0::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
{
	LoadDynaSave0Param param;

	for (auto &p : params)
	{
		if (p.first == "period")
			param.period = std::stod(p.second);
		if (p.first == "amplitude")
			param.amplitude = std::stod(p.second);

	}

	target.param = param;


		aris::core::Matrix mat0(1, LoadTotalParas + 6, JointMatrix.estParasL0Yang);
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

}
LoadDynaSave0::LoadDynaSave0(const std::string &name) :Plan(name)
{

	command().loadXmlStr(
		"<Command name=\"LoadDynaSave0\">"
		"	<GroupParam>"
		"		<Param name=\"period\"default=\"20.0\"/>"
		"		<Param name=\"amplitude\" default=\"0.2\"/>"
		"	</GroupParam>"
		"</Command>");

}




struct LoadDynaSave1Param
{
	double period;
	double amplitude;

};
auto LoadDynaSave1::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
{
	LoadDynaSave1Param param;

	for (auto &p : params)
	{
		if (p.first == "period")
			param.period = std::stod(p.second);
		if (p.first == "amplitude")
			param.amplitude = std::stod(p.second);

	}

	target.param = param;

	
			aris::core::Matrix mat2(1, 10, JointMatrix.LoadParas);
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
LoadDynaSave1::LoadDynaSave1(const std::string &name) :Plan(name)
{

	command().loadXmlStr(
		"<Command name=\"LoadDynaSave1\">"
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
	std::string gk_path;
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



