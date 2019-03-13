﻿#include "cplan.h"
#include <math.h>
using namespace std;
using namespace aris::plan;
using namespace aris::dynamic;

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
    //求运动学反解需要调用求解器solverpool，0是反解，1是正解，kinPos是位置反解，kinVel是速度反解
    if(target.model->solverPool()[0].kinPos() == 0 && target.count %500 ==0)target.master->mout()<< "kin failed"<<std::endl;
    return p.total_time - target.count;
}

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
    double dec;
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
	aris::Size total_count = 1;
	double begin_pos = 0.0;//局部变量最好赋一个初始值
	if (target.count == 1)
	{
		begin_pos = controller->motionAtAbs(5).targetPos();
	}
	aris::plan::moveAbsolute(target.count, begin_pos, p.pt, p.vel / 1000, p.acc / 1000 / 1000, p.dec / 1000 / 1000, pt, v, a, t_count);
	target.model->motionPool().at(5).setMp(pt);
	total_count = std::max(total_count, t_count);
	return p.total_time - target.count;
}
MoveTroute::MoveTroute(const std::string &name) :Plan(name)
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
}


/// \brief 结构体申明
///结构体MoveFileParam， 用于读取.txt文件的点位置信息，并控制机器人按照文件中的位置运动
struct MoveFileParam
{
    int total_time;
     
    double vel;
    double acc;
    double dec;
    vector<double> pt;
    int choose;
	string file;
};

int n = 25; // n代表txt文档中数据的列数
vector<vector<double>  > POS(n);
vector<vector<double>  > POS2;
//读取指定文件夹的所有文件名，并存储在容器vector files[]中；

void getFiles2(string path, vector<string>& files)  
{
	std::vector<std::filesystem::path> loc_files;
	
	for (auto &p : std::filesystem::directory_iterator(path))  //遍历元素，p为局部变量"log"；
	{
		if(p.is_regular_file())loc_files.push_back(p.path());//检查p是否常规的文件，.path()是将路径输出的函数；
		//std::cout << p.path() << "\n";
		//p.path();
		//files.push_back(p.path());
	}
	//先输出正常的文件顺序
	//for (auto &p : loc_files)   
	//{
	//	std::cout << p << std::endl;  
	//}
	//按照修改时间排序
	std::sort(loc_files.begin(), loc_files.end(), [](const std::filesystem::path &p1, const std::filesystem::path &p2)->bool   //lambda函数，匿名
	{
		//return p1.string() < p2.string();	
		//p1和p2是指待排序的vector中的前两个对象，此处采用"冒泡排序"算法，进行挨个对比排序
		return std::filesystem::last_write_time(p1) < std::filesystem::last_write_time(p2);//返回布尔结果，true或者false;
	});	
	std::cout << "---------------------------------------------------------------------------" << std::endl << std::endl;
	//输入排序后的结果
	for (auto &p : loc_files)
	{
		std::cout << p << std::endl;
	}
	//flies和local_files是两个类的成员，所以要重新使用local_files对files赋值；
	files.clear();
	for (auto &p : loc_files) files.push_back(p.string());//p是path,给path写类的人写了一个函数string()；
	std::cout << std::endl;
}

//以下只能在C++中使用
//void getFiles(string path, vector<string>& files)
//{
//	//文件句柄  
//	long   hFile = 0;
//	//文件信息，声明一个存储文件信息的结构体  
//	struct _finddata_t fileinfo;
//	string p;//字符串，存放路径
//	if ((hFile = _findfirst(p.assign(path).append("\\*").c_str(), &fileinfo)) != -1)//若查找成功，则进入
//	{
//		do
//		{
//			//如果是目录,迭代之（即文件夹内还有文件夹）  
//			if ((fileinfo.attrib &  _A_SUBDIR))
//			{
//				//文件名不等于"."&&文件名不等于".."
//					//.表示当前目录
//					//..表示当前目录的父目录
//					//判断时，两者都要忽略，不然就无限递归跳不出去了！
//				if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0)
//					getFiles(p.assign(path).append("\\").append(fileinfo.name), files);
//			}
//			//如果不是,加入列表  
//			else
//			{
//				files.push_back(p.assign(path).append("\\").append(fileinfo.name));
//			}
//		} while (_findnext(hFile, &fileinfo) == 0);
//		//_findclose函数结束查找
//		_findclose(hFile);
//	}
//}
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
	p.file = params.at("file");
    aris::core::Matrix mat = target.model->calculator().calculateExpression(params.at("pt"));

	if (mat.size() != 6)
	{
		throw std::runtime_error("The value of mat.size() is not 6");
	}
    p.pt.resize(mat.size());
    std::copy(mat.begin(), mat.end(), p.pt.begin());//begin和end都是标准的vector的迭代器

    for (int j = 0; j < n; j++)
    {
        POS[j].clear();
    }
	//string site = "C:/Users/qianch_kaanh_cn/Desktop/myplan/src/rokae/" + p.file;
    

	//char * filePath = "/home/kaanh/Desktop/build-kaanh-Desktop_Qt_5_11_2_GCC_64bit-Default/log/";//自己设置目录 
	string filePath = "C:/Users/qianch_kaanh_cn/Desktop/build_qianch/log/";
	vector<string> files;
	//获取该路径下的所有文件  
	getFiles2(filePath, files);

	char str[30];
	int size = files.size();
	for (int i = 0; i < size; i++)
	{
		cout << files[i].c_str() << endl;
	}

	//以下确认所读文件中是否包含字符串“movePQB”;
	vector<int> effpos;//整形数组，记录files中有效的文件对应的位置；
	for (int i = 0; i < files.size(); i++)
	{
		string a = "movePQB";
		string::size_type idx;
		idx = files[i].find(a); //在files[i]中查找字符串a;
		if (idx != string::npos)
		{
			effpos.push_back(i);
		}
	}
	p.file = files[effpos.back()].c_str();
	cout << p.file;
	

	//清理前50个文件
	//std::filesystem::space_info devi = std::filesystem::space("log");
	//std::cout << ".        Capacity       Free      Available\n"
	//	<< "/log:   " << devi.capacity << "   "
	//	<< devi.free << "   " << devi.available << '\n';
	////如果可用内存小于10g;
	//if (devi.available < 10737418240)
	//{
	//	for (int i = 0; i < 50; i++)
	//	{
	//		std::filesystem::remove(files[i]);
	//	}		
	//}


	//string site = "/home/kaanh/Desktop/build-kaanh-Desktop_Qt_5_11_2_GCC_64bit-Default/" + p.file;
	//p.file=log\rt_log--2019-01-21--17-02-45--movePQB.txt,里面有log目录
	//string site = "C:/Users/qianch_kaanh_cn/Desktop/build_qianch/" + p.file;
	string site = p.file;
	//以下定义读取log文件的输入流oplog;
    ifstream oplog;
    int cal = 0;
	oplog.open(site);
	//以下检查是否成功读取文件；
	if (!oplog)
	{
		cout << "fail to open the file" << endl;
		throw std::runtime_error("fail to open the file");
		//return -1;//或者抛出异常。
	}
    while (!oplog.eof())
    {
        for (int j = 0; j < n; j++)
        {
            double data;
            oplog >> data;
			POS[j].push_back(data);          
        }
    }
    oplog.close();
    oplog.clear();
    for (int j = 0; j < n; j++)
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
	    NOT_RUN_EXECUTE_FUNCTION;
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
	aris::Size return_value = 0;
	static double begin_pos[6] = { 0.0,0.0,0.0,0.0,0.0,0.0 }; // 局部变量最好赋一个初始值;

	if (target.count == 1)
	{
		for (int i = 0; i < 6; i++)
		{
			// 在第一个周期走梯形规划复位，获取6个电机初始位置；
			begin_pos[i] = controller->motionAtAbs(i).actualPos();
		}
	}
	//choose==0的情况下机器人本体复位
	if (p.choose == 0)
	{
		for (int i = 0; i < 6; i++)
		{
			// 在第一个周期走梯形规划复位
			aris::plan::moveAbsolute(target.count, begin_pos[i], p.pt[i], p.vel / 1000, p.acc / 1000 / 1000, p.dec / 1000 / 1000, ptt, v, a, t_count);
			controller->motionAtAbs(i).setTargetPos(ptt);
			total_count = std::max(total_count, t_count);
		}
		return_value = target.count > total_count ? 0 : 1;
	}
	//choose==1的情况下机器人走到数据文件的第一个位置
	else if (p.choose == 1)
	{
		for (int i = 0; i < 6; i++)
		{
			// 在第一个周期走梯形规划复位
            aris::plan::moveAbsolute(target.count, begin_pos[i], POS[3 * i ][0], p.vel / 1000, p.acc / 1000 / 1000, p.dec / 1000 / 1000, ptt, v, a, t_count);
			controller->motionAtAbs(i).setTargetPos(ptt);
			total_count = std::max(total_count, t_count);
		}
		return_value = target.count > total_count ? 0 : 1;
	}
	//choose==2的情况下机器人本体按照示教轨迹来走
	else if (p.choose == 2)
	{
		//正式走示教的轨迹
		for (int i = 0; i < 6; i++)
		{
            controller->motionAtAbs(i).setTargetPos(POS[3 * i][target.count]);//从列表的第二行开始走起，第一行在choose=1已经到达了
		}
		return_value = target.count > POS[0].size() - 2 ? 0 : 1;  //-2的原因在于a程序从文件数据第二行开始走b实时核程序判断结束是“先斩后奏”，所以要减1；
	}
	//输出6个轴的实时位置log文件
	auto &lout = controller->lout();
	for (int i = 0; i < 6; i++)
	{
        lout << POS[3*i][target.count - 1] << endl;//第一列数字必须是位置
	}
	auto &cout = controller->mout();
	//以下验证读取文件的正确性；
	if (target.count % 500 == 0)
	{
		for (int i = 0; i < 6; i++)
		{
			cout << "POS[0][0]" << POS[0][0] << "    " << "POS[3][0]" << POS[3][0] << "    " << "size:" << POS[0].size() << "    " << begin_pos[i] << "    " << p.pt[i] << ",  ";
		}
		cout << std::endl;
	}
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
           // "		<m type=\"Param\" default=\"59815\"/>"  // 行数
            //"		<n type=\"Param\" default=\"24\"/>"
            "		<vel type=\"Param\" default=\"0.04\"/>"
            "		<acc type=\"Param\" default=\"0.08\"/>"
            "		<dec type=\"Param\" default=\"0.08\"/>"
            "		<choose type=\"Param\" default=\"0\"/>"
            "		<pt type=\"Param\" default=\"{0.0,0.0,0.0,0.0,0.0,0.0}\"/>"
			"		<file type=\"Param\" default=\"1.txt\" abbreviation=\"f\"/>"
            "	</group>"
            "</mvFi>");
    }




struct RemoveFileParam
{
	int total_time;
};

auto RemoveFile::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
{
	RemoveFileParam p;
	p.total_time = std::stoi(params.at("total_time"));
	
	//string site = "C:/Users/qianch_kaanh_cn/Desktop/myplan/src/rokae/" + p.file;

    string filePath = "/home/kaanh/Desktop/build-kaanh-Desktop_Qt_5_11_2_GCC_64bit-Default/log/";//自己设置目录
    //char * filePath = "/home/kaanh/Desktop/build-kaanh-Desktop_Qt_5_11_2_GCC_64bit-Default/log/";//自己设置目录
    //string filePath = "C:/Users/qianch_kaanh_cn/Desktop/build_qianch/log/";
	vector<string> files;
	//获取该路径下的所有文件  
    files.clear();
	getFiles2(filePath, files);

	std::filesystem::space_info devi = std::filesystem::space("log");
	std::cout << ".        Capacity       Free      Available\n"
		<< "/log:   " << devi.capacity << "   "
		<< devi.free << "   " << devi.available << '\n';
	//如果可用内存小于10g;
    if (devi.available < 10737418240 * 4)
	{
        std::cout<<files[0].size()<<"  ";
        std::cout<<files[0].substr(72)<<"  ";
        for (int i = 0; i < 30; i++)
		{
            std::filesystem::remove(files[i]);
            //std::cout<<"success123456";
            std::cout<<files[0]<<"  ";
		}
	}
	//std::filesystem::remove(files[0]);
	target.param = p;
	target.option =
		//aris::plan::Plan::USE_TARGET_POS |
		aris::plan::Plan::NOT_CHECK_VEL_MIN |
		aris::plan::Plan::NOT_CHECK_VEL_MAX |
		aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
		aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER_AT_START |
		aris::plan::Plan::NOT_CHECK_VEL_FOLLOWING_ERROR |
		aris::plan::Plan::NOT_CHECK_VEL_CONTINUOUS_AT_START | // 开始不检查速度连续
        aris::plan::Plan::NOT_CHECK_VEL_CONTINUOUS |
	    NOT_RUN_EXECUTE_FUNCTION;
}

auto RemoveFile::executeRT(PlanTarget &target)->int
{
	auto controller = dynamic_cast<aris::control::EthercatController *>(target.master);
    std::cout<<"before"<<std::endl;
    auto &p = std::any_cast<MoveFileParam&>(target.param);
    std::cout<<"after"<<std::endl;

	return 0;
}

RemoveFile::RemoveFile(const std::string &name) :Plan(name)
{
	command().loadXmlStr(
		"<rmFi>"
		"	<group type=\"GroupParam\" default_child_type=\"Param\">"
		"	    <total_time type=\"Param\" default=\"5000\"/>" // 默认5000	   
		"	</group>"
		"</rmFi>");
}











//再mod中复现pq轨迹
struct replayParam
{
	std::vector<std::vector<double>> pq_convert;
	int col;
};

auto replay::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
{
	replayParam param;

	string site = "C:/Users/qianch_kaanh_cn/Desktop/data/rt_log--2019-02-27--14-15-05--6.txt";
	//以下定义读取log文件的输入流oplog; 
	ifstream oplog;

	oplog.open(site);
	//以下检查是否成功读取文件；
	if (!oplog)
	{
		cout << "fail to open the file" << endl;
		throw std::runtime_error("fail to open the file");
		//return -1;//或者抛出异常。
	}
	while (!oplog.eof())
	{
		for (int j = 0; j < 25; j++)
		{
			double data;
			oplog >> data;
			POS[j].push_back(data);
		}
	}
	oplog.close();
	oplog.clear();
	for (int j = 0; j < n; j++)
	{
		POS[j].pop_back();
	}
	param.col = POS[0].size();

	param.pq_convert = std::vector<std::vector<double>>(param.col, std::vector<double>(7, 0));
	//直接从POS中提取pq验证是否能复现
	for (int i = 0; i < param.col; i++)
	{
		for (int j = 0; j < 7; j++)
		{
			param.pq_convert[i][j] = POS[j + 18][i];
		}
	}



	target.param = param;
	target.option =
		//aris::plan::Plan::USE_TARGET_POS |
		aris::plan::Plan::NOT_CHECK_VEL_MIN |
		aris::plan::Plan::NOT_CHECK_VEL_MAX |
		aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
		aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER_AT_START |
		aris::plan::Plan::NOT_CHECK_VEL_FOLLOWING_ERROR |
		aris::plan::Plan::NOT_CHECK_VEL_CONTINUOUS_AT_START | // 开始不检查速度连续
		aris::plan::Plan::NOT_CHECK_VEL_CONTINUOUS;
	NOT_RUN_EXECUTE_FUNCTION;
}


auto replay::executeRT(PlanTarget &target)->int
{
	auto controller = dynamic_cast<aris::control::EthercatController *>(target.master);
	auto &p = std::any_cast<replayParam&>(target.param);

	target.model->generalMotionPool().at(0).setMpq(p.pq_convert[target.count - 1].data());

	target.model->solverPool()[0].kinPos();

	//std::cout << "p.pq_convert[target.count - 1].size():" << p.pq_convert[target.count - 1].size() << std::endl;
	std::cout << "pq_convert1_7" << p.pq_convert[target.count - 1][0] << "   " << p.pq_convert[target.count - 1][1] << "   " << p.pq_convert[target.count - 1][2] << std::endl;
	std::cout << "pq_convert1_7:   " << p.pq_convert[target.count - 1][3] << "   " << p.pq_convert[target.count - 1][4] << "   " << p.pq_convert[target.count - 1][5] << "   " << p.pq_convert[target.count - 1][6] << std::endl;
	//std::cout << "target.count:  " << target.count << std::endl;
	//if(p.pq_convert[1][1]==pq[1][1] && p.pq_convert[2][2] == pq[2][2] && p.pq_convert[3][3] == pq[3][3])
		//std::cout << "convert successfully in executeRT" << std::endl;
	return p.col - target.count;

}


replay::replay(const std::string &name) :Plan(name)
{
	command().loadXmlStr(
		"<rp>"
		"	<group type=\"GroupParam\" default_child_type=\"Param\">"
		//"	    <total_time type=\"Param\" default=\"5000\"/>" // 默认5000
		//"		<vel type=\"Param\" default=\"0.04\"/>"
		//"		<acc type=\"Param\" default=\"0.08\"/>"
		//"		<dec type=\"Param\" default=\"0.08\"/>"
		//"		<choose type=\"Param\" default=\"0\"/>"
		//"		<pt type=\"Param\" default=\"{0.0,0.0,0.0,0.0,0.0,0.0}\"/>"
		//"		<file type=\"Param\" default=\"1.txt\" abbreviation=\"f\"/>"
		"	</group>"
		"</rp>");
}



//直接给14维数组
auto load_pq2(aris::Size count, aris::Size &start_count)->std::array<double, 14>

{
	std::array<double, 14> temp = { -0.122203,0.386206,0.0139912,-0.492466,0.474288,0.511942,0.520041,0,0,0,0,0,0,0 };
	std::array<double, 7> targetpos1 = { -0.122203,0.386206,0.0139912,-0.492466,0.474288,0.511942,0.520041 };
	std::array<double, 7> targetpos2 = { -0.122203,0.466206,0.0139912,-0.492466,0.474288,0.511942,0.520041 };
	std::array<double, 7> targetpos3 = { 0.162203,0.466206,0.0139912,-0.492466,0.474288,0.511942,0.520041 };
	std::array<double, 7> targetpos4 = { 0.162203,0.386206,0.0,-0.492466,0.474288,0.511942,0.520041 };
	std::array<double, 7> targetpos5 = { -0.122203,0.386206,0.0,-0.492466,0.474288,0.511942,0.520041 };
	double vel = 0.04, acc = 0.08, dec = 0.08, v, a;
	aris::Size t_count;
	static aris::Size count_last = 0, count_last2 = 0, count_last3 = 0, count_last4 = 0;//上个轨迹完成共消耗的count ,每个阶段单独的时间
	if (count == start_count)
	{
		for (int i = 0; i < 3; i++)
		{
			aris::plan::moveAbsolute(1, targetpos1[i], targetpos2[i], vel / 1000, acc / 1000 / 1000, dec / 1000 / 1000, temp[i], v, a, t_count);
			count_last = std::max(t_count, count_last);
		}
		for (int i = 0; i < 3; i++)
		{
			aris::plan::moveAbsolute(1, targetpos2[i], targetpos3[i], vel / 1000, acc / 1000 / 1000, dec / 1000 / 1000, temp[i], v, a, t_count);
			count_last2 = std::max(t_count, count_last2);
		}
		for (int i = 0; i < 3; i++)
		{
			aris::plan::moveAbsolute(1, targetpos3[i], targetpos4[i], vel / 1000, acc / 1000 / 1000, dec / 1000 / 1000, temp[i], v, a, t_count);
			count_last3 = std::max(t_count, count_last3);
		}
		for (int i = 0; i < 3; i++)
		{
			aris::plan::moveAbsolute(1, targetpos4[i], targetpos5[i], vel / 1000, acc / 1000 / 1000, dec / 1000 / 1000, temp[i], v, a, t_count);
			count_last4 = std::max(t_count, count_last4);
		}
	}

	//1
	if (count <= count_last + start_count)
	{
		//aris::plan::moveAbsolute(target.count, begin_pos[i], POS[3 * i][0], p.vel / 1000, p.acc / 1000 / 1000, p.dec / 1000 / 1000, ptt, v, a, t_count);
		for (int i = 0; i < 3; i++)
		{
			aris::plan::moveAbsolute(count - start_count, targetpos1[i], targetpos2[i], vel / 1000, acc / 1000 / 1000, dec / 1000 / 1000, temp[i], v, a, t_count);
		}
	}
	//2
	else if (count <= count_last2 + count_last + start_count && count > count_last + start_count)
	{
		for (int i = 0; i < 3; i++)
		{
			aris::plan::moveAbsolute(count - start_count, targetpos2[i], targetpos3[i], vel / 1000, acc / 1000 / 1000, dec / 1000 / 1000, temp[i], v, a, t_count);
		}
	}
	//3
	else if (count <= count_last3 + count_last2 + count_last + start_count && count > count_last2 + count_last + start_count)
	{
		for (int i = 0; i < 3; i++)
		{
			aris::plan::moveAbsolute(count - start_count, targetpos3[i], targetpos4[i], vel / 1000, acc / 1000 / 1000, dec / 1000 / 1000, temp[i], v, a, t_count);
		}
	}
	//4
	else if (count < count_last4 + count_last3 + count_last2 + count_last + start_count && count >count_last3 + count_last2 + count_last + start_count)
	{
		for (int i = 0; i < 3; i++)
		{
			aris::plan::moveAbsolute(count - start_count, targetpos4[i], targetpos5[i], vel / 1000, acc / 1000 / 1000, dec / 1000 / 1000, temp[i], v, a, t_count);
		}
	}
	return temp;
}



std::vector<std::vector<double>> pq(7);
//在prepare函数中提前读好txt文档中的数据
auto load_pq5()->void
{
	//将文件中的数据读取到POS中，共25列；
	//string site = "/home/kaanh/Desktop/build-kaanh-Desktop_Qt_5_11_2_GCC_64bit-Default/log/rt_log--2019-02-20--16-34-25--4.txt";
	//std::cout << "start1" << std::endl;
	//string site = "C:/Users/kevin/Desktop/qch/rt_log--2019-02-20--16-34-25--4.txt";
	string site = "C:/Users/qianch_kaanh_cn/Desktop/data/rt_log--2019-02-27--14-15-05--6.txt";
	//string site = "C:/Users/qianch_kaanh_cn/Desktop/data/rt_log--2019-02-27--14-13-05--7.txt";
	//以下定义读取log文件的输入流oplog;
	ifstream oplog;
	oplog.open(site);
	//以下检查是否成功读取文件；
	if (!oplog)
	{
		cout << "fail to open the file" << endl;
		throw std::runtime_error("fail to open the file");
		//return -1;//或者抛出异常。
	}
	for (int i = 0; i < 25; i++)
	{
		POS[i].clear();
	}
	//清空pq；
	for (int i = 0; i < 7; i++)
	{
		pq[i].clear();
	}

	while (!oplog.eof())
	{
		for (int j = 0; j < n; j++)
		{
			double data;
			oplog >> data;
			POS[j].push_back(data);
		}
	}
	oplog.close();
	oplog.clear();
	for (int j = 0; j < n; j++)
	{
		POS[j].pop_back();
	}
	int row = POS[0].size();//总行数
	std::cout << "POS[18][0]" << POS[18][0] << "   " << "POS[19][0]" << POS[19][0] << "   " << "POS[20][0]" << POS[20][0] << std::endl;



	for (int i = 0; i < row - 1; i++)
	{
		//轨迹上切线的向量
		double tangent[3] = { POS[18][i + 1] - POS[18][i],POS[19][i + 1] - POS[19][i] ,POS[20][i + 1] - POS[20][i] };
		double tangL = sqrt(tangent[0] * tangent[0] + tangent[1] * tangent[1] + tangent[2] * tangent[2]);
		//点距离小于5mm时


		if (tangL < 0.005)
		{
			POS[18][i + 1] = POS[18][i];
			POS[19][i + 1] = POS[19][i];
			POS[20][i + 1] = POS[20][i];
		}
		else
		{
			double x[3] = { 1, 0, 0 };
			double y[3] = { 0, 1, 0 };
			double z[3] = { 0, 0, -1 };
			//求取切线与y轴的公法线向量
			double vert[3] = { 0,0,0 };
			//叉乘
			s_c3(tangent, y, vert);
			//求单位向量；
			double sq = sqrt(vert[0] * vert[0] + vert[1] * vert[1] + vert[2] * vert[2]);


			std::cout << "sq" << sq << std::endl;

			double vert0[3] = { 0,0,0 };
			for (int j = 0; j < 3; j++)
			{
				vert0[j] = vert[j] / sq;
			}
			//先求叉乘，再求arcsin
			double vertsin[3] = { 0,0,0 };
			s_c3(vert0, z, vertsin);
			double vsmo = sqrt(vertsin[0] * vertsin[0] + vertsin[1] * vertsin[1] + vertsin[2] * vertsin[2]);
			//求点乘，由于是单位向量，点乘即角度
			//double dot = s_vv(3, vert0, z);
			//对dot做出限制
			vsmo = std::max(vsmo, -1.0);
			vsmo = std::min(vsmo, 1.0);
			//注意theta的正负性,与Z轴成180度左右；
			double theta = asin(vsmo);
			//欧拉角是2pi,2pi,theta		
			//相对
			double pe[6] = { POS[18][i] ,POS[19][i] ,POS[20][i],atan(1) * 2 ,atan(1) * 2 , -theta };

			//绝对
			//double pe[6] = { POS[18][i] ,POS[19][i] ,POS[20][i], atan(1) * 2,-atan(1) * 2 , theta };
			//vector <vector <double>>pm(4, std::vector<double>(4, 0.0));
			double pm[4][4];
			//double pm[16];
			//输出pm
			//相对
			s_pe2pm(pe, pm[0], "323");//必须是二位数组的第一个地址
			//绝对
			//s_pe2pm(pe, pm[0], "312");
			//s_pe2pm(pe, *pm, "323");
			double pq0[7] = { 0,0,0,0,0,0,0 };
			//输出pq
			s_pm2pq(pm[0], pq0);
			for (int s = 0; s < i - pq[0].size() + 1; s++)
			{
				for (int j = 0; j < 7; j++)
				{
					pq[j].push_back(pq0[j]);
				}
			}
		}
	}
	std::cout << "pq[0][0]" << pq[0][0] << "    " << "pq[1][0]" << pq[1][0] << "    " << "pq[2][0]" << pq[2][0] << "pq[3][0]" << pq[3][0] << "    " << "pq[4][0]" << pq[4][0] << "    " << "pq[5][0]" << pq[5][0] << "    " << "pq[6][0]" << pq[6][0] << std::endl;
	std::cout << "pq size:" << pq[0].size() << std::endl;
}





//函数pq7是将pq转化为带速度的参数temp
auto load_pq7(aris::Size count, aris::Size &start_count)->std::array<double, 14>
{
	//定义新的14列容器temp
	std::array<double, 14> temp = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
	int pqsize = pq[0].size();
	std::cout << "pqsize" << pqsize << std::endl;
	if (count - start_count < pqsize - 1)
	{
		std::cout << "count- start_count" << count - start_count << "pq[0][count- start_count]" << pq[0][count - start_count] << "   " << "pq[1][count- start_count]" << pq[1][count - start_count] << std::endl;
		for (int j = 0; j < 7; j++)
		{
			temp[j] = pq[j][count - start_count];
		}
		if (count == start_count)
		{
			for (int s = 0; s < 7; s++)
			{
				temp[s + 7] = 0;
			}
		}
		//计算速度
		else
		{
			for (int s = 0; s < 7; s++)
			{
				temp[s + 7] = (pq[s][count - start_count + 1] - pq[s][count - start_count]) / 1000;
			}
		}
	}
	else
	{
		for (int j = 0; j < 7; j++)
		{
			temp[j] = pq[j][pqsize - 1];
			temp[j + 7] = (pq[j][pqsize - 1] - pq[j][pqsize - 2]) / 1000;
		}
	}
	return temp;
}


///函数pq9是采用廖能超的方法从POS开始截取数据
//记录上一个切线向量的z轴值
//double tangentz_last;
//第9个方程是数据不经过处理，从POS中采取间隔一定距离的点来采集数据
auto load_pq9()->void
{
	//将文件中的数据读取到POS中，共25列；
	//string site = "/home/kaanh/Desktop/build-kaanh-Desktop_Qt_5_11_2_GCC_64bit-Default/log/rt_log--2019-02-20--16-34-25--4.txt";
	
	//string site = "C:/Users/kevin/Desktop/qch/rt_log--2019-02-20--16-34-25--4.txt";
	std::cout << "now is on pq9" << std::endl;
	string site = "C:/Users/qianch_kaanh_cn/Desktop/data/rt_log--2019-02-28--23-25-52--11.txt";
	
	//以下定义读取log文件的输入流oplog;
	ifstream oplog;

	vector<double> theta2;
	theta2.clear();

	oplog.open(site);
	//以下检查是否成功读取文件；
	if (!oplog)
	{
		cout << "fail to open the file" << endl;
		throw std::runtime_error("fail to open the file");
		//return -1;//或者抛出异常。
	}
	for (int i = 0; i < 25; i++)
	{
		POS[i].clear();
	}
	//清除pq，防止多次调用累积增加
	for (int i = 0; i < 7; i++)
	{
		pq[i].clear();
	}

	while (!oplog.eof())
	{
		for (int j = 0; j < n; j++)
		{
			double data;
			oplog >> data;
			POS[j].push_back(data);
		}
	}
	oplog.close();
	oplog.clear();
	for (int j = 0; j < n; j++)
	{
		POS[j].pop_back();
	}
	
	///以下定义二位数组POS2，是从POS中间隔取点
	//先定义取点的间隔p_space
	/*
	int p_space = 1;//间隔50个点取一次数据
	int loop2 = (int)POS[0].size() / p_space;
	POS2 = std::vector<std::vector<double>>(25, std::vector<double>(loop2, 0));
	*/

	//std::vector<std::vector<double>>POS2(25);
	
	/*for (int i = 0; i < 25; i++)
	{
		POS2[i].clear();
	}*/
	int step_wave = 2;//滤波间隔；
	int row = POS[0].size();//总行数
	for (int j = 0; j < row - step_wave; j++)
	{
		double X = 0;
		for (int i = j; i < j + step_wave; i++)
		{
			X = X + POS[18][i];
		}
		X = X / step_wave;
		double Z = 0;
		for (int i = j; i < j + step_wave; i++)
		{
			Z = Z + POS[20][i];
		}
		Z = Z / step_wave;


		POS[18][j] = X;
		POS[20][j] = Z;
		/*if (j == row - 4)
		{
			POS2[18][j+1] = X;

			POS2[18][j+2] = X;
			POS2[18][j+3] = X;
			POS2[20][j + 1] = Z;
			POS2[20][j + 2] = Z;
			POS2[20][j + 3] = Z;
		}*/
		//std::cout << "POS2[20][j]" << POS2[20][j] << std::endl;
	}
	double time = 0;
    double tangent[3] = { 0,0,0 };
	for (int i = 0; i < POS[0].size() - step_wave; i++)
	{		
		if (i <= 5000)
		{
			time = time + 0.001;
			tangent[0] = 0.0001;
			tangent[2]=0.0001*cos(time);
		}
		if(i>5000&&i<=10000)
		{
			time = time - 0.001;
			tangent[0] = -0.0001;
			tangent[2]=-0.0001*cos(time);
		}
		if (i>10000&&i <= 15000)
		{
			time = time + 0.001;
			tangent[0] = 0.0001;
			tangent[2] = 0.0001*cos(time);
		}
		if (i > 15000 && i <= 20000)
		{
			time = time - 0.001;
			tangent[0] = -0.0001;
			tangent[2] = -0.0001*cos(time);
		}
		if (i > 20000&&i <= 25000)
		{
			time = time + 0.001;
			tangent[0] = 0.0001;
			tangent[2] = 0.0001*cos(time);
		}
		if (i > 25000 && i <= 30000)
		{
			time = time - 0.001;
			tangent[0] = -0.0001;
			tangent[2] = -0.0001*cos(time);
		}
		if (i > 30000 && i <= 35000)
		{
			time = time + 0.001;
			tangent[0] = 0.0001;
			tangent[2] = 0.0001*cos(time);
		}
		if (i > 35000 && i <= 40000)
		{
			time = time - 0.001;
			tangent[0] = -0.0001;
			tangent[2] = -0.0001*cos(time);
		}
		

		if (i == 0)
		{

			POS[18][i] = POS[18][0];
			POS[19][i] = POS[19][0];
			POS[20][i] = POS[19][0];
		}
		else
		{
			POS[18][i] = POS[18][i - 1] + tangent[0];
			POS[19][i] = POS[19][i - 1];
		    POS[20][i] = POS[20][i - 1] + tangent[2];
		}

	
		double tangL = sqrt(tangent[0] * tangent[0] + tangent[1] * tangent[1] + tangent[2] * tangent[2]);
		if (tangL < 5 * 10e-27)//-6************************************************************************
			continue;
		else
		{
			 
			//int coe;
			double x[3] = { 1, 0, 0 };
			double y[3] = { 0, 100000, 0 };
			double z[3] = { 0, 0, -1 };
			//求取切线与y轴的公法线向量
			double vert[3] = { 0,0,0 };
			//叉乘
			s_c3(tangent, y, vert);
			//先求出垂直向量的模，再除以模长求出单位向量；


			double sq = sqrt(vert[0] * vert[0] + vert[1] * vert[1] + vert[2] * vert[2]);

			//std::cout << "sq" << sq << std::endl;

			double vert0[3] = { 0,0,0 };
			
			for (int j = 0; j < 3; j++)
			{
				vert0[j] = vert[j] / sq;
			}
			//先求叉乘，再求arcsin

			//求点乘，由于是单位向量，点乘即角度
			//double dot = s_vv(3, vert0, z);
			//对dot做出限制
			//std::cout << " vert0[2]: " << vert0[2] << "    " << "vert0[0]:  " << vert0[0] << std::endl;
			//注意theta的正负性,与Z轴成180度左右；

			double theta = atan(vert0[0] / vert0[2]);
			theta2.push_back(theta);
			//std::cout << "theta:  " << theta << std::endl;
			//欧拉角是2pi,2pi,theta		
			//相对
			//double pe[6] = { POS2[18][i] ,POS2[19][i] ,POS2[20][i],atan(1) * 2 ,atan(1) * 2 , -theta };
			double pe[6] = { POS[18][i] ,POS[19][i] ,POS[20][i],atan(1) * 2 ,atan(1) * 2 , theta };

			//绝对
			//double pe[6] = { POS[18][i] ,POS[19][i] ,POS[20][i], atan(1) * 2,-atan(1) * 2 , theta };
			//vector <vector <double>>pm(4, std::vector<double>(4, 0.0));
			double pm[4][4];
			//double pm[16];
			//输出pm
			//相对
			s_pe2pm(pe, pm[0], "323");//必须是二位数组的第一个地址
			//绝对
			//s_pe2pm(pe, pm[0], "312");
			//s_pe2pm(pe, *pm, "323");
			double pq0[7] = { 0,0,0,0,0,0,0 };
			//输出pq
			s_pm2pq(pm[0], pq0);

			for (int j = 0; j < 7; j++)
			{
				pq[j].push_back(pq0[j]);
			}
			int xxxx = 0;
		}
	}
	//POS2完成数据转换，数据没有更改
	//int row = POS2[0].size();//总行数
	//以下再POS2中对数值做平均化滤波
	//for (int j = 0; j < row - 1; j++)
	//{
	//	//std::cout << "POS2[20][j]未转换" << POS2[20][j] << std::endl;
	//}	
	//for (int j = 0; j < row; )
	//{
	//	double A = (POS2[20][j] + POS2[20][j + 1]+ POS2[20][j + 2]+ POS2[20][j + 3]) / step_wave;//z值；
	//	POS2[20][j] = A;
	//	POS2[20][j + 1] = A;
	//	POS2[20][j + 2] = A;
	//	POS2[20][j + 3] = A;
	//	j = j + step_wave;
	//}
	//std::cout << "POS2[18][0]" << POS2[18][0] << "   " << "POS2[19][0]" << POS2[19][0] << "   " << "POS2[20][0]" << POS2[20][0] << std::endl;
	//int step_i = 1;
	
	std::cout << "pq[0][0]" << pq[0][0] << "    " << "pq[1][0]" << pq[1][0] << "    " << "pq[2][0]" << pq[2][0] << "pq[3][0]" << pq[3][0] << "    " << "pq[4][0]" << pq[4][0] << "    " << "pq[5][0]" << pq[5][0] << "    " << "pq[6][0]" << pq[6][0] << std::endl;
	//std::cout << "pq size:" << pq[0].size() << std::endl;
	ofstream outfile("C:/Users/qianch_kaanh_cn/Desktop/data/outfile.txt");
	if (!outfile)
	{
		cout << "Unable to open otfile";
		exit(1); // terminate with error
	}
	for (int k = 0; k < theta2.size(); k++)
	//for (int k = 0; k < POS[0].size(); k++)
	{
		outfile << POS[18][k] << "  " << POS[19][k] << "  " << POS[20][k] << "  " << pq[3][k]<< "  " << pq[4][k] << "  " << pq[5][k] << "  " << pq[6][k] << "  " << theta2[k] << endl;
		cout <<  "success outfile "  << endl;
		//cout << data[k][0] << " " << data[k][1] << endl;
	}
	outfile.close();
}




struct MoveinModelParam
{
	//int total_time;
	std::vector<std::vector<double>> pq_convert;
};
//int col = pq[0].size();
//std::vector<std::vector<double>> pq_convert(col, std::vector<double>(7, 0));
auto MoveinModel::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
{
	//load_pq5();
	load_pq9();
	//得到二维数组pq;
	//获取行数
	int col = pq[0].size();
	MoveinModelParam param;
	param.pq_convert = std::vector<std::vector<double>>(col, std::vector<double>(7, 0));
	//将pq行与列转换
	for (int i = 0; i < col; i++)
	{
		for (int j = 0; j < 7; j++)
		{
			param.pq_convert[i][j] = pq[j][i];
		}
	}
	if (param.pq_convert[1][1] == pq[1][1] && param.pq_convert[2][2] == pq[2][2] && param.pq_convert[3][3] == pq[3][3])
		std::cout << "convert successfully" << std::endl;
	

	target.param = param;
	target.option =
		aris::plan::Plan::USE_TARGET_POS |
		aris::plan::Plan::NOT_CHECK_VEL_MIN |
		aris::plan::Plan::NOT_CHECK_VEL_MAX |
		aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
		aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER_AT_START |
		aris::plan::Plan::NOT_CHECK_VEL_FOLLOWING_ERROR |
		aris::plan::Plan::NOT_CHECK_VEL_CONTINUOUS_AT_START | // 开始不检查速度连续
		aris::plan::Plan::NOT_CHECK_VEL_CONTINUOUS;
		NOT_RUN_EXECUTE_FUNCTION;
}

auto MoveinModel::executeRT(PlanTarget &target)->int
{
	auto controller = dynamic_cast<aris::control::EthercatController *>(target.master);
	auto &p = std::any_cast<MoveinModelParam&>(target.param);
	
	target.model->generalMotionPool().at(0).setMpq(p.pq_convert[target.count-1].data());

	target.model->solverPool()[0].kinPos();
	auto &lout = controller->lout();
	for (int i = 0; i < 6; i++)
	{
		lout << target.model->motionPool().at(i).mp() << " ";
	}
	lout << std::endl;
	
	
	//std::cout << "p.pq_convert[target.count - 1].size():" << p.pq_convert[target.count - 1].size() << std::endl;
	std::cout << "pq_convert1_7" << p.pq_convert[target.count - 1][0] <<"   "<< p.pq_convert[target.count - 1][1] << "   " << p.pq_convert[target.count - 1][2] << std::endl;
	//std::cout << "pq_convert1_7:   " << p.pq_convert[target.count - 1][3] << "   " << p.pq_convert[target.count - 1][4] << "   " << p.pq_convert[target.count - 1][5] << "   " << p.pq_convert[target.count - 1][6] << std::endl;
	//std::cout << "target.count:  " << target.count << std::endl;
	//if(p.pq_convert[1][1]==pq[1][1] && p.pq_convert[2][2] == pq[2][2] && p.pq_convert[3][3] == pq[3][3])
		//std::cout << "convert successfully in executeRT" << std::endl;
	return pq[0].size()- target.count;
	
}


MoveinModel::MoveinModel(const std::string &name) :Plan(name)
{
	command().loadXmlStr(
		"<mvinmod>"
		"	<group type=\"GroupParam\" default_child_type=\"Param\">"
		//"	    <total_time type=\"Param\" default=\"5000\"/>" // 默认5000
		//"		<vel type=\"Param\" default=\"0.04\"/>"
		//"		<acc type=\"Param\" default=\"0.08\"/>"
		//"		<dec type=\"Param\" default=\"0.08\"/>"
		//"		<choose type=\"Param\" default=\"0\"/>"
		//"		<pt type=\"Param\" default=\"{0.0,0.0,0.0,0.0,0.0,0.0}\"/>"
		//"		<file type=\"Param\" default=\"1.txt\" abbreviation=\"f\"/>"
		"	</group>"
		"</mvinmod>");
}



