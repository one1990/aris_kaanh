#include "cplan.h"
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
*/

//直接给14维数组
auto load_pq2(aris::Size count, aris::Size &start_count)->std::array<double, 14>
{
    std::array<double, 14> temp = { -0.122203,0.396206,0.0139912,-0.492466,0.474288,0.511942,0.520041,0,0,0,0,0,0,0 };
    std::array<double, 7> targetpos1 = { -0.122203,0.396206,0.0139912,-0.492466,0.474288,0.511942,0.520041 };
	std::array<double, 7> targetpos2 = { -0.122203,0.466206,0.0139912,-0.492466,0.474288,0.511942,0.520041 };
	std::array<double, 7> targetpos3 = { 0.162203,0.466206,0.0139912,-0.492466,0.474288,0.511942,0.520041 };
    std::array<double, 7> targetpos4 = { 0.162203,0.396206,0.0139912,-0.492466,0.474288,0.511942,0.520041 };
    std::array<double, 7> targetpos5 = { -0.122203,0.396206,0.0139912,-0.492466,0.474288,0.511942,0.520041 };
    double vel = 0.1, acc = 0.2, dec = 0.2, v, a;
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


///函数pq9是采用廖能超的方法从POS开始截取数据
//记录上一个切线向量的z轴值
//double tangentz_last;
//第9个方程是数据不经过处理，从POS中采取间隔一定距离的点来采集数据
auto load_pq9()->void
{
	//将文件中的数据读取到POS中，共25列；
	//string site = "/home/kaanh/Desktop/build-kaanh-Desktop_Qt_5_11_2_GCC_64bit-Default/log/rt_log--2019-02-20--16-34-25--4.txt";
	//std::cout << "start1" << std::endl;
	//string site = "C:/Users/kevin/Desktop/qch/rt_log--2019-02-20--16-34-25--4.txt";
	std::cout << "now is on pq9" << std::endl;
	string site = "C:/Users/qianch_kaanh_cn/Desktop/data/rt_log--2019-02-28--23-25-52--11.txt";
	//string site = "C:/Users/qianch_kaanh_cn/Desktop/data/rt_log--2019-02-27--14-13-05--7.txt";
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
	int row = POS[0].size();//总行数
	
	int step_wave = 2;//滤波间隔；
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

		//double X = (POS2[18][j] + POS2[18][j + 1] + POS2[18][j + 2] + POS2[18][j + 3]) / step_wave;//x值；
		//double Z = (POS2[20][j] + POS2[20][j + 1] + POS2[20][j + 2] + POS2[20][j + 3]) / step_wave;//z值；
		//std::cout << "A" << A << std::endl;
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

	for (int i = 0; i< POS[0].size()- step_wave; i++)
	{
		double tangent[3] = { POS[18][i + step_wave] - POS[18][i],POS[19][i + step_wave] - POS[19][i] ,POS[20][i + step_wave] - POS[20][i] };
		double tangL = sqrt(tangent[0] * tangent[0] + tangent[1] * tangent[1] + tangent[2] * tangent[2]);
		if (tangL < 5 * 10e-17)//-6************************************************************************
			continue;
		else
		{
			/*for (int i = 0; i < 25; i++)
			{
				POS2[i].push_back(POS[i][j]);
				POS2[i].push_back(POS[i][j+1]);
			}
			j++;*/
			//轨迹上切线的向量
			//double tangent[3] = { POS[18][i + step_wave] - POS[18][i],POS[19][i + step_wave] - POS[19][i] ,POS[20][i + step_wave] - POS[20][i] };
			//double tangL = sqrt(tangent[0] * tangent[0] + tangent[1] * tangent[1] + tangent[2] * tangent[2]);
			//定义与Z轴方向的欧拉角系数coe
			//int coe;
			double x[3] = { 1, 0, 0 };
			double y[3] = { 0, 1, 0 };
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
		outfile << POS[18][k] << "  " << POS[19][k] << "  " << POS[20][k] << "  " << POS[21][k]<< "  " << POS[22][k] << "  " << POS[23][k] << "  " << POS[24][k] << "theta2:   " << theta2[k] << endl;
		cout <<  "success outfile "  << endl;
		//cout << data[k][0] << " " << data[k][1] << endl;
	}
	outfile.close();
}


//采用akima插值方法
//std::vector<double> p1(row), p2(row), p3(row);
std::vector<double> p1, p2, p3;
int row = 0;
auto load_pq10()->void
{
	//将文件中的数据读取到POS中，共25列；
	std::cout << "now is on pq10" << std::endl;
	string site = "C:/Users/qianch_kaanh_cn/Desktop/data/rt_log--2019-02-28--23-25-52--11.txt";
	//string site = "C:/Users/qianch_kaanh_cn/Desktop/data/rt_log--2019-02-27--14-13-05--7.txt";
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
	row = POS[0].size();//总行数
	cout << "row:" << row << endl;
	std::vector<double> p1, p2, p3;
	p1 = std::vector<double> (row);
	p2 = std::vector<double>(row);
	p3 = std::vector<double>(row);

	s_akima(row, POS[18].data(), POS[20].data(), p1.data(), p2.data(), p3.data());
	//double A[5] = { 1,3,7,10,15 };
	//double B[5] = { 1,6,-1,8,4 };
	//s_akima(5, A, B, p1.data(), p2.data(), p3.data());
	int step_wave = 2;//滤波间隔；

	/*double Kz1 = s_akima_at(5, A, B, p1.data(), p2.data(), p3.data(), 2, '1');
	double Kz2 = s_akima_at(5, A, B, p1.data(), p2.data(), p3.data(), 4.5, '1');
	double Kz3 = s_akima_at(5, A, B, p1.data(), p2.data(), p3.data(), 7, '1');
	double Kz4 = s_akima_at(5, A, B, p1.data(), p2.data(), p3.data(), 8.5, '1');
	double Kz5 = s_akima_at(5, A, B, p1.data(), p2.data(), p3.data(), 12, '1');
	std::cout << "Kz1" << Kz1 << "   " << "Kz2" << Kz2 << "   " << "Kz3" << Kz3 << "Kz4" << Kz4 << "   " << "Kz5" << Kz5 << std::endl;*/

	
	for (int i = 0; i < POS[0].size() - step_wave; i++)
	{

		double time = i * 0.001;
		//Kz是Z轴的斜率
		double Kz = s_akima_at(row, POS[18].data(), POS[20].data(), p1.data(), p2.data(), p3.data(), POS[18][0]+i* 0.00001, '1');
		cout <<"time:"<<time<< "  Kz:   " << Kz << endl;
		double tangent[3] = { 0.00001,0 ,0.00001*Kz };
		if (i <= 3000)
			tangent[0] = 0.0001;

		if (cos(2 * time) > 0)
			tangent[0] = -0.0001;
		else
			tangent[0] = 0.0001;

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
			double tangL = sqrt(tangent[0] * tangent[0] + tangent[1] * tangent[1] + tangent[2] * tangent[2]);
			if (tangL < 5 * 10e-17)//-6************************************************************************
				continue;
			else
			{
				/*for (int i = 0; i < 25; i++)
				{
					POS2[i].push_back(POS[i][j]);
					POS2[i].push_back(POS[i][j+1]);
				}
				j++;*/
				//轨迹上切线的向量
				//double tangent[3] = { POS[18][i + step_wave] - POS[18][i],POS[19][i + step_wave] - POS[19][i] ,POS[20][i + step_wave] - POS[20][i] };
				//double tangL = sqrt(tangent[0] * tangent[0] + tangent[1] * tangent[1] + tangent[2] * tangent[2]);
				//定义与Z轴方向的欧拉角系数coe
				//int coe;
				double x[3] = { 1, 0, 0 };
				double y[3] = { 0, 1, 0 };
				double z[3] = { 0, 0, -1 };
				//求取切线与y轴的公法线向量
				double vert[3] = { 0,0,0 };
				//叉乘
				s_c3(tangent, y, vert);
				//先求出垂直向量的模，再除以模长求出单位向量；
				double sq = sqrt(vert[0] * vert[0] + vert[1] * vert[1] + vert[2] * vert[2]);
				std::cout << "sq" << sq << std::endl;
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
			}
		}
	}
		
		//int step_i = 1;
		std::cout << "pq[0].size():  " << pq[0].size() << endl;
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
			outfile << POS[18][k] << "  " << POS[19][k] << "  " << POS[20][k] << "  " << POS[21][k] << "  " << POS[22][k] << "  " << POS[23][k] << "  " << POS[24][k] << " theta2:   " << theta2[k] << endl;
			cout << "success outfile " << endl;
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
	load_pq10();
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
		//NOT_RUN_EXECUTE_FUNCTION;
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


//FMovePath --pq={      }
vector <vector<double>> pq_real(7);
struct FMovePathParam
{
	vector<vector<double>>pq;//是二位数组吗
};
auto FMovePath::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
{
	aris::core::Calculator c;
	auto m = c.calculateExpression(params.at("pq"));
	int row_n = m.m();//计算矩阵的总行数；
	std::cout << "row_n:  " << row_n << endl;
	std::cout << "m(1,1):  " << m(1, 1) << endl;
	//pq1~7是对应的pq的7个数组 
	vector<double> pq1,pq2,pq3,pq4,pq5,pq6,pq7;
	pq1.clear();
	pq2.clear();
	pq3.clear();
	pq4.clear();
	pq5.clear();
	pq6.clear();
	pq7.clear();
	for (int i = 0; i < row_n; ++i)
	{
		pq1.push_back(m(i, 0));
		pq2.push_back(m(i, 1));
		pq3.push_back(m(i, 2));
		pq4.push_back(m(i, 3));
		pq5.push_back(m(i, 4));
		pq6.push_back(m(i, 5));
		pq7.push_back(m(i, 6));
	}	
	//pqi进行归一化
	for (int i = 0; i < row_n; i++)
	{
		double norm2 = sqrt(pq4[i] * pq4[i] + pq5[i] * pq5[i] + pq6[i] * pq6[i]+ pq7[i] * pq7[i]);
		pq4[i] = pq4[i] / norm2;
		pq5[i] = pq5[i] / norm2;
		pq6[i] = pq6[i] / norm2;
		pq7[i] = pq7[i] / norm2;
	}
	//调整四元数的方向
	for (int i = 0; i < row_n -1; i++)
	{
		double eleA[4] = { pq4[i] ,pq5[i],pq6[i],pq7[i] };
		double eleB[4] = { pq4[i + 1] ,pq5[i + 1],pq6[i + 1],pq7[i + 1] };
		double result_ele = pq4[i] * pq4[i + 1] + pq5[i] * pq5[i + 1] + pq6[i] * pq6[i + 1] + pq7[i] * pq7[i + 1];
		if (result_ele < 0)
		{
			pq4[i + 1] = -pq4[i + 1];
			pq5[i + 1] = -pq5[i + 1];
			pq6[i + 1] = -pq6[i + 1];
		}
	}
	//定义akima函数的参数p1,p2p3
	std::vector<double> p1, p2, p3;	
	p1 = std::vector<double>(pq1.size());
	p2 = std::vector<double>(pq1.size());
	p3 = std::vector<double>(pq1.size());
	//孟给定我的点的数目
	int numM = pq1.size();
	//定义和时间相关的数组
	vector<double> time;
	int runtime = 5;//5s内走完所有的点
	for (int i = 0; i < pq1.size(); i++)
	{
		time.push_back((i + 1)*(runtime/numM));
	}
	s_akima(pq1.size(), time.data(), pq1.data(), p1.data(), p2.data(), p3.data());//?????????????p1.data()
	//获取插值之后的pq数值
	for (int i = 0; i < 7; i++)
	{
		pq_real[i].clear();//将pq_real先清空
	}
	//注意统一单位为s，否则求导之后系数有问题
	for (int i = 0; i < runtime * 1000; i++)
	{
		double x_t = 1/ runtime/ 1000 * (i+1);
		double c1 = s_akima_at(pq1.size(), time.data(), pq1.data(), p1.data(), p2.data(), p3.data(), x_t, '0');
		double c2 = s_akima_at(pq1.size(), time.data(), pq2.data(), p1.data(), p2.data(), p3.data(), x_t, '0');
		double c3 = s_akima_at(pq1.size(), time.data(), pq3.data(), p1.data(), p2.data(), p3.data(), x_t, '0');
		double c4 = s_akima_at(pq1.size(), time.data(), pq4.data(), p1.data(), p2.data(), p3.data(), x_t, '0');
		double c5 = s_akima_at(pq1.size(), time.data(), pq5.data(), p1.data(), p2.data(), p3.data(), x_t, '0');
		double c6 = s_akima_at(pq1.size(), time.data(), pq6.data(), p1.data(), p2.data(), p3.data(), x_t, '0');
		double c7 = s_akima_at(pq1.size(), time.data(), pq7.data(), p1.data(), p2.data(), p3.data(), x_t, '0');
		pq_real[0].push_back(c1);
		pq_real[1].push_back(c2);
		pq_real[2].push_back(c3);
		pq_real[3].push_back(c4);
		pq_real[4].push_back(c5);
		pq_real[5].push_back(c6);
		pq_real[6].push_back(c7);
	}
	std::cout << "run succssful" << std::endl;
	std::cout << "pq_real[0].size():  "<< pq_real[0].size() << std::endl;

	/*for (auto &p : params)
	{
		if (p.first == "setpqJF")
		{
			auto pqarray = target.model->calculator().calculateExpression(p.second);
			std::array<double, 7> temp;
			std::copy(pqarray.begin(), pqarray.end(), temp.begin());
			setpqJF.store(temp);
		}
		else if (p.first == "setpqJFB")
		{
			auto pqarray = target.model->calculator().calculateExpression(p.second);
			std::array<double, 7> temp;
			std::copy(pqarray.begin(), pqarray.end(), temp.begin());
			setpqJFB.store(temp);
		}
		else if (p.first == "setpqPQB")
		{
			auto pqarray = target.model->calculator().calculateExpression(p.second);
			std::array<double, 14> temp = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
			std::copy(pqarray.begin(), pqarray.end(), temp.begin());
			setpqPQB.store(temp);
			which_func == 1;
			func[1] = load_pq1;
		}
		else if (p.first == "which_func")
		{
			which_func = std::stoi(p.second);
			if (which_func == 1)
			{
				func[1] = load_pq1;
				one_time_counter = false;
			}
			else if (which_func == 2)
			{
				func[1] = load_pq2;
				one_time_counter = true;
			}
			else if (which_func == 3)
			{
				func[1] = load_pq3;
				one_time_counter = true;
			}
			else if (which_func == 7)
			{
				func[1] = load_pq7;
				one_time_counter = true;
			}
		}
	}*/
	target.option = aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION;
}
FMovePath::FMovePath(const std::string &name):Plan(name)
{
	command().loadXmlStr(
		"<FMovePath>"
		"	<group type=\"GroupParam\" default_child_type=\"Param\">"
		//"		<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"setpqJFB\">"
		"			<pq default=\"{0.42,0.0,0.55,0,0,0,1}\"/>"
		//"			<pq default=\"{0.42,0.0,0.55,0,0,0,1}\"/>"
		//"			<setpqJFB default=\"{0.42,0.0,0.55,0,0,0,1}\"/>"
		//"			<setpqPQB default=\"{0.42,0.0,0.55,0,0,0,1}\"/>"
		//"			<which_func default=\"1\"/>"
		//"		</unique>"
		"	</group>"
		"</FMovePath>");
}



//pq_real(7)是全局变量
//函数pq7是将pq转化为带速度的参数temp
auto load_pq7(aris::Size count, aris::Size &start_count)->std::array<double, 14>
{
	//定义新的14列容器temp
	std::array<double, 14> temp = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
	int pqsize = pq_real[0].size();
	std::cout << "pqsize" << pqsize << std::endl;
	if (count - start_count < pqsize - 1)
	{
		std::cout << "count- start_count" << count - start_count << "pq_real[0][count- start_count]" << pq_real[0][count - start_count] << "   " << "pq_real[1][count- start_count]" << pq_real[1][count - start_count] << std::endl;
		for (int j = 0; j < 7; j++)
		{
			temp[j] = pq_real[j][count - start_count];
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
				temp[s + 7] = (pq_real[s][count - start_count + 1] - pq_real[s][count - start_count]) / 1000;
			}
		}
	}
	else
	{
		for (int j = 0; j < 7; j++)
		{
			temp[j] = pq_real[j][pqsize - 1];
			temp[j + 7] = (pq_real[j][pqsize - 1] - pq_real[j][pqsize - 2]) / 1000;
		}
	}
	return temp;
}