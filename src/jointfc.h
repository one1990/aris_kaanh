#ifndef JOINTFC_H_
#define JOINTFC_H_

#include <aris.hpp>
#include <iostream>
#include <fstream>
#include <typeinfo>
#include <vector>
#include <memory>
#include <atomic>
#include <string> 
#include <algorithm>


	class JointDyna : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		explicit JointDyna(const std::string &name = "JointDyna");
		ARIS_REGISTER_TYPE(JointDyna);
	};


	class JointDynaSave : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;

		explicit JointDynaSave(const std::string &name = "JointDynaSave");
		ARIS_REGISTER_TYPE(JointDynaSave);
	};



	class JointTest : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;

		explicit JointTest(const std::string &name = "JointTest");
		ARIS_REGISTER_TYPE(JointTest);
	};

	class DragTeach : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;

		explicit DragTeach(const std::string &name = "DragTeach");
		ARIS_REGISTER_TYPE(DragTeach);
	};




	class LoadDyna : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		explicit LoadDyna(const std::string &name = "LoadDyna");
		ARIS_REGISTER_TYPE(LoadDyna);
	};

	class LoadDynaSave0 : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;

		explicit LoadDynaSave0(const std::string &name = "LoadDynaSave0");
		ARIS_REGISTER_TYPE(LoadDynaSave0);
	};

	class LoadDynaSave1 : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;

		explicit LoadDynaSave1(const std::string &name = "LoadDynaSave1");
		ARIS_REGISTER_TYPE(LoadDynaSave1);
	};

	class SaveYYbase : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;

		explicit SaveYYbase(const std::string &name = "SaveYYbase");
		ARIS_REGISTER_TYPE(SaveYYbase);
	};


	class SaveFile : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;

		explicit SaveFile(const std::string &name = "SaveFile");
		ARIS_REGISTER_TYPE(SaveFile);
	};




	/*
	//与Meng交换数据
	class GetError : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		explicit GetError(const std::string &name = "GetError");
		ARIS_REGISTER_TYPE(GetError);
	};*/




#endif