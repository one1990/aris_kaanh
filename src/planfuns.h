#ifndef PLANFUNS_H_
#define PLANFUNS_H_

#include <aris.hpp>
#include <iostream>
#include <fstream>
#include <typeinfo>
#include <vector>
#include <memory>
#include <atomic>
#include <string> 
#include <algorithm>

namespace PLANGK
{
	class planconfig
	{
	public:
	
		planconfig();
	    void DoubleSVelocity(const double jMax, const double aMax, const double vMax, const double sMax, double* T, double &vlim, double &alim);
		void RealSVelocity(const double t, const double jMax, const double aMax, const double vMax, const double sMax, const double* Tt, const double vlim, const double alim, double &st, double &vt, double &at, double &jt);
	};

}



#endif