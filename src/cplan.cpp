#include <aris.h>
#include <iostream>
#include <fstream>
#include <typeinfo>
#include <vector>
#include "cplan.h"
using namespace std;
using namespace aris::plan;

/// \brief
struct MoveCParam
{
    int total_time;
    int left_time;
    double radius; //圆形规划半径
    double detal;//圆弧轨迹增量
    double theta;//计算的中间变量
};


auto MoveCircle::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
{
    MoveCParam p;
    p.total_time = std::stoi(params.at("total_time"));
    p.radius = std::stod(params.at("radius"));
    p.detal = target.model->calculator().calculateExpression(params.at("detal")).toDouble();//如果直接用stod，则detal中除以5000的分母直接被忽略了
    p.left_time = 0;
    target.param = p;
    target.option =
        //用这段话可以不用将model的轨迹赋值到controller里面，系统直接调用model中的反解计算结果
        aris::plan::Plan::USE_TARGET_POS |
        aris::plan::Plan::NOT_CHECK_VEL_FOLLOWING_ERROR |
        aris::plan::Plan::NOT_CHECK_VEL_CONTINUOUS_AT_START |
        aris::plan::Plan::NOT_CHECK_VEL_CONTINUOUS;
}
auto MoveCircle::executeRT(PlanTarget &target)->int
{
    auto controller = dynamic_cast<aris::control::EthercatController *>(target.master);
    auto &p = std::any_cast<MoveCParam&>(target.param);

    static double beginpq[7];
    if (target.count == 1)
    {
        //获取当前模型的位置
        target.model->generalMotionPool()[0].getMpq(beginpq);//generalMotionPool()[0]一个末端
    }
    //定义一个变量，四元数，前三个代表位置
    double pq[7];
    //将获取的机器人位置赋值给变量
    std::copy_n(beginpq, 7, pq);
    //对变量的第一个参数进行运动规划
    double theta = 1.0*(1 - std::cos(1.0*target.count / p.total_time * 1 * aris::PI));
    pq[1] = beginpq[1] + p.radius*(std::cos(-aris::PI/2+1.0* theta * 5 * 2 * aris::PI)) + p.detal * theta * 5000;//Y轴，水平轴,走完5个圆
    pq[2] = beginpq[2] + p.radius*(std::sin(-aris::PI/2+1.0*theta * 5 * 2 * aris::PI)) + p.radius;//Z轴竖直轴
    if(target.count %500 ==0)target.master->mout()<< pq[1] <<"  "<<pq[2]<< std::endl;
    //将变量的值赋值给model中模型的末端位置
    target.model->generalMotionPool()[0].setMpq(pq);
    //target.model->generalMotionPool()[2].setMpq(pq);
    //求运动学反解需要调用求解器solverpool，kinPos是位置反解，kinVel是速度反解
    if(target.model->solverPool()[0].kinPos() == 0 && target.count %500 ==0)target.master->mout()<< "kin failed"<<std::endl;
    //只有一个末端位置，所以只有一个求解器，所以以下语句不对
    //target.model->solverPool()[2].kinPos();
    //for(int i = 0; i<6;++i)
    //{
     //   controller->motionPool()[i].setTargetPos(target.model->motionPool()[i].mp());
    //}
    return p.total_time - target.count;
}
//auto MoveCircle::collectNrt(PlanTarget &target)->void {}
MoveCircle::MoveCircle(const std::string &name) :Plan(name)
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


//末端空间的梯形轨迹控制
struct MoveTParam
{
    int total_time;
    double vel;
    double acc;
    double dec;//v0是梯形轨迹最终速度
    double pt;
};

    auto MoveTroute::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
    {
        MoveTParam p;
        p.total_time = std::stoi(params.at("total_time"));
        p.vel = std::stod(params.at("vel"));
        p.acc = std::stod(params.at("acc"));
        p.dec = std::stod(params.at("dec"));
        p.pt = std::stod(params.at("pt"));

        target.param = p;
        target.option =
            //用这段话可以不用将model的轨迹赋值到controller里面，系统直接调用model中的反解计算结果，如果
            //不用这个命令，那么需要用for循环将model中的反解计算结果赋值到controller里面
            aris::plan::Plan::USE_TARGET_POS |
            aris::plan::Plan::NOT_CHECK_VEL_FOLLOWING_ERROR |
            aris::plan::Plan::NOT_CHECK_VEL_CONTINUOUS_AT_START | //开始不检查速度连续
            aris::plan::Plan::NOT_CHECK_VEL_CONTINUOUS;
    }
    auto MoveTroute::executeRT(PlanTarget &target)->int
    {
        auto controller = dynamic_cast<aris::control::EthercatController *>(target.master);
        auto &p = std::any_cast<MoveTParam&>(target.param);
        double pt, v, a;
        aris::Size t_count;
        aris::Size total_count=1;
        double begin_pos=0.0;//局部变量最好赋一个初始值
        if (target.count == 1)
        {
            //target.model->generalMotionPool()[0].getMpq(beginpq);
            begin_pos = controller->motionAtAbs(5).targetPos();
        }
        aris::plan::moveAbsolute(target.count, begin_pos ,p.pt , p.vel / 1000, p.acc / 1000 / 1000, p.dec / 1000 / 1000, pt, v, a, t_count);
        target.model->motionPool().at(5).setMp(pt);//motionpool驱动器的池子、数组
        //target.model->motionPool().at(5).setMv(v * 1000);
        total_count = std::max(total_count, t_count);


        //if (target.model->solverPool()[0].kinPos() == 0 && target.count % 500 == 0)target.master->mout() << "kin failed" << std::endl;

        return p.total_time - target.count;
    }
    MoveTroute::MoveTroute(const std::string &name):Plan(name)
    {
        command().loadXmlStr(
            "<mvTT>"
            "	<group type=\"GroupParam\" default_child_type=\"Param\">"
            "	    <total_time type=\"Param\" default=\"5000\"/>" //默认5000
            "		<pt type=\"Param\" default=\"0.1\"/>"
            "		<vel type=\"Param\" default=\"0.04\"/>"
            "		<acc type=\"Param\" default=\"0.08\"/>"
            "		<dec type=\"Param\" default=\"0.08\"/>"
            "	</group>"
"</mvTT>");
//"</moveEAP>");
    }


/// \brief 结构体申明
///结构体MoveFileParam， 用于读取.txt文件的点位置信息，并控制机器人按照文件中的位置运动
struct MoveFileParam
{
    int total_time;
    int m, n;  // m,n代表txt文档中数据的行数和列数
    double vel;
    double acc;
    double dec;// v0是梯形轨迹最终速度
    vector<double> pt;
    int choose;
	string file;
};
//vector<vector<double>  > POS(72, std::vector<double>(700, 0.0));
/// 申明二维数组用于存储文件中的位置信息
//static double POS[59815][72];
//static double P1[59815];
//vector <double> P1;
vector<vector<double>  > POS(24);
/// \brief 类MoveFile申明
/// 在类MoveFile中,完成对现有的.txt文件中的位置数据的提取，通过程序控制机器人的各关节按照数据位置变化来运动
/// ### 类MoveFile
/// + 实时核的准备函数prepairNrt
/// + 实时核函数executeRT
/// + 类构造函数MoveFile实时读取xml文件信息

/// \brief 实时核的准备函数prepairNrt
/// @param &params 头文件std中的类map，键的类型是string，值的类型也是string
/// @param PlanTarget 命名空间aris::plan中定义的结构体，target是它的引用
/// @return 返回值为空
auto MoveFile::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
{
    MoveFileParam p;
    p.total_time = std::stoi(params.at("total_time"));
    p.vel = std::stod(params.at("vel"));
    p.acc = std::stod(params.at("acc"));
    p.dec = std::stod(params.at("dec"));
    p.choose = std::stoi(params.at("choose"));
	p.file = std::any_cast<string>(params.at("file"));
	//以下三段代码从xml中读取数组pt信息；
    aris::core::Matrix mat = target.model->calculator().calculateExpression(params.at("pt"));
    p.pt.resize(mat.size());
    std::copy(mat.begin(), mat.end(), p.pt.begin());//begin和end都是标准的vector的迭代器
    //for(int i=0;i<6;i++)
    //{
    //    std::cout<<p.pt[i]<<", ";
    //}
    //std::cout<<std::endl;

    // 读取txt文件
    p.m = std::stoi(params.at("m"));
    p.n = std::stoi(params.at("n"));
    //std::fill_n(*POS, 59815 * 72, 0.0);

    for (int j = 0; j < p.n; j++)
    {
        POS[j].clear();
    }

    ifstream oplog;
    int cal = 0;
	
    //oplog.open("C:\\Users\\qianch_kaanh_cn\\Desktop\\myplan\\src\\rokae\\rt_log--2019-01-10--14-29-07--5--1.txt");
    //oplog.open("C:/Users/qianch_kaanh_cn/Desktop/myplan/src/rokae/1.txt");
    //string site = "C:/Users/qianch_kaanh_cn/Desktop/myplan/src/rokae/" + p.file;
    string site = "/home/kaanh/Desktop/myplan/src/rokae/" + p.file;
	oplog.open(site);
	//oplog.open(p.file);
	if (!oplog)
	{
		cout << "fail to open the file" << endl;
		//return -1;//或者抛出异常。
	}
    while (!oplog.eof())
    {
        for (int j = 0; j < p.n; j++)
        {
            // oplog >> POS[i][j];
            double data;
            oplog >> data;

            POS[j].push_back(data);
        }
    }

    oplog.close();
    oplog.clear();
    for (int j = 0; j < p.n; j++)
    {
        POS[j].pop_back();
    }
    target.param = p;
    target.option =
        //aris::plan::Plan::USE_TARGET_POS |
        aris::plan::Plan::NOT_CHECK_VEL_MIN |
        aris::plan::Plan::NOT_CHECK_VEL_MAX |
        aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
        aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER_AT_START |
        aris::plan::Plan::NOT_CHECK_VEL_FOLLOWING_ERROR |
        aris::plan::Plan::NOT_CHECK_VEL_CONTINUOUS_AT_START | // 开始不检查速度连续
        aris::plan::Plan::NOT_CHECK_VEL_CONTINUOUS;


}
/// \brief 实时核函数executeRT
/// @param PlanTarget 命名空间aris::plan中定义的结构体，target是它的引用
/// @return 返回值为空
auto MoveFile::executeRT(PlanTarget &target)->int
{
    auto controller = dynamic_cast<aris::control::EthercatController *>(target.master);
    auto &p = std::any_cast<MoveFileParam&>(target.param);
    double ptt, v, a;
    aris::Size t_count;
    aris::Size total_count = 1;
    aris::Size return_value=0;
    static double begin_pos[6] = { 0.0,0.0,0.0,0.0,0.0,0.0 }; // 局部变量最好赋一个初始值;
	
    if (target.count == 1)
    {
        //target.model->generalMotionPool()[0].getMpq(beginpq);
        for (int i = 0; i < 6; i++)
        {
            // 在第一个周期走梯形规划复位
            begin_pos[i] = controller->motionAtAbs(i).actualPos();// 获取6个电机初始位置
            //target.model->motionPool().at(i).setMp(controller->motionAtAbs(i).actualPos());
            //if (!target.model->solverPool().at(1).kinPos())return -1;
            //begin_pos[i] = target.model->motionPool().at(i).mp();
        }
    }
	//choose==0的情况下机器人本体复位
    if (p.choose == 0)
    {
        for (int i = 0; i < 6; i++)
        {
            // 在第一个周期走梯形规划复位
            aris::plan::moveAbsolute(target.count, begin_pos[i], p.pt[i], p.vel / 1000, p.acc / 1000 / 1000, p.dec / 1000 / 1000, ptt, v, a, t_count);
            //target.model->motionPool().at(i).setMp(ptt);// motionpool驱动器的池子、数组
            //target.model->motionPool().at(5).setMv(v * 1000);
            controller->motionAtAbs(i).setTargetPos(ptt);
            total_count = std::max(total_count, t_count);
        }
        return_value = target.count>total_count ?0:1;
    }
	//choose==1的情况下机器人走到数据文件的第一个位置
	else if (p.choose == 1)
	{
		for (int i = 0; i < 6; i++)
		{
			// 在第一个周期走梯形规划复位
			aris::plan::moveAbsolute(target.count, begin_pos[i], POS[4 * i][0], p.vel / 1000, p.acc / 1000 / 1000, p.dec / 1000 / 1000, ptt, v, a, t_count);
			controller->motionAtAbs(i).setTargetPos(ptt);
			total_count = std::max(total_count, t_count);
		}
		return_value = target.count > total_count ? 0 : 1;
	}
	//choose==2的情况下机器人本体按照示教轨迹来走
    else if (p.choose == 2)
     {
        //target.master->mout() << POS[0][target.count] << std::endl;
        //target.model->motionPool().at(0).setMp(POS[0][target.count]);// motionpool驱动器的池子、数组
        //target.model->motionPool().at(1).setMp(POS[3][target.count]);
        //target.model->motionPool().at(2).setMp(POS[6][target.count]);
        //target.model->motionPool().at(3).setMp(POS[9][target.count]);
        //target.model->motionPool().at(4).setMp(POS[12][target.count]);
        //target.model->motionPool().at(5).setMp(POS[15][target.count]);
		//if (begin_pos[0] != POS[0][0] || begin_pos[12] != POS[12][0] || begin_pos[24] != POS[24][0] || begin_pos[36] != POS[36][0] || begin_pos[48] != POS[48][0] || begin_pos[60] != POS[60][0])
		//{
		//	for (int i = 0; i < 6; i++)
		//	{
		//		// 在第一个周期走梯形规划复位
		//		aris::plan::moveAbsolute(target.count, begin_pos[i], POS[12*i][0], p.vel / 1000, p.acc / 1000 / 1000, p.dec / 1000 / 1000, ptt, v, a, t_count);
		//		//target.model->motionPool().at(i).setMp(ptt);// motionpool驱动器的池子、数组
		//		//target.model->motionPool().at(5).setMv(v * 1000);
		//		controller->motionAtAbs(i).setTargetPos(ptt);
		//		total_count = std::max(total_count, t_count);
		//	}
		//}
		//正式走示教的轨迹
			for (int i = 0; i < 6; i++)
        {
            //controller->motionAtAbs(i).setTargetPos(POS[3*i][target.count]);//3个一组
            controller->motionAtAbs(i).setTargetPos(POS[4 * i][target.count]);//从列表的第二行开始走起，第一行在choose=1已经到达了
        }
        return_value = target.count>POS[0].size()-2?0:1;  //-2的原因在于a程序从文件数据第二行开始走b实时核程序判断结束是“先斩后奏”，所以要减一 

    }

    //if (!target.model->solverPool().at(1).kinPos())return -1;

    auto &lout = controller->lout();
    for (int i = 0; i < 6;i++)
    {
        lout << POS[4*i][target.count-1] << endl;
    }

    //for (Size i = 0; i < 6; i++)
    //{
    //	lout << controller->motionAtAbs(i).targetPos() << ",";
    //	lout << controller->motionAtAbs(i).actualPos() << ",";
    //	lout << controller->motionAtAbs(i).actualVel() << ",";
    //	lout << controller->motionAtAbs(i).actualCur() << ",";
    //}
    //lout << std::endl;
    auto &cout = controller->mout();
    if (target.count % 500 == 0)
    {
        for (int i = 0; i < 6;i++)
        {
            cout << "POS[0][0]"<< POS[0][0]<< "    " <<"size:"<<POS[0].size()<<"    "<<begin_pos[i]<<"    "<< p.pt[i]<< ",  ";
        }
         cout << std::endl;
        //target.master->mout() << "POS[1].size()"<<POS[0].size() << " "<<POS.size() << "  POS[1].size()  " << std::endl;
    }
    //target.model-`>motionPool().at(5).setMv(p.POS[;][63])
    //target.model->motionPool().at(5).setMv(v * 1000);
    //total_count = std::max(total_count, t_count);
    //输出log位置文件

    /*auto &cout = controller->mout();
    for (int i = 0; i < 6; i++)
    {
        cout << POS[3*i][count] << ",";
    }*/

    //if (target.model->solverPool()[0].kinPos() == 0 && target.count % 500 == 0)target.master->mout() << "kin failed" << std::endl;

    //return total_count - target.count;
    return return_value;

}

/// + 类构造函数MoveFile实时读取xml文件信息
///
MoveFile::MoveFile(const std::string &name) :Plan(name)
    {
        command().loadXmlStr(
            "<mvFi>"
            "	<group type=\"GroupParam\" default_child_type=\"Param\">"
            "	    <total_time type=\"Param\" default=\"5000\"/>" // 默认5000
            "		<m type=\"Param\" default=\"59815\"/>"  // 行数
            "		<n type=\"Param\" default=\"24\"/>"
            "		<vel type=\"Param\" default=\"0.04\"/>"
            "		<acc type=\"Param\" default=\"0.08\"/>"
            "		<dec type=\"Param\" default=\"0.08\"/>"
            "		<choose type=\"Param\" default=\"0\"/>"
            "		<pt type=\"Param\" default=\"{0.0,0.0,0.0,0.0,0.0,0.0}\"/>"
			"		<file type=\"Param\" default=\"1.txt\" abbreviation=\"f\"/>"
            "	</group>"
            "</mvFi>");
    }

