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

namespace traplan
{
	class planconfig
	{
	public:

		planconfig();
		void DoubleSVelocity(const double jMax, const double aMax, const double vMax, const double sMax, double* T, double &vlim, double &alim);
		void RealSVelocity(const double t, const double jMax, const double aMax, const double vMax, const double sMax, const double* Tt, const double vlim, const double alim, double &st, double &vt, double &at, double &jt);
	};


	auto sCurveParam(double q0, double q1, double vmax, double amax, double jmax, double &taj, double &ta, double &tv, double &tdj, double &td, double &vlim, double &alima, double &alimd)->void;
	auto sCurve(size_t t, double q0, double q1, double vmax, double amax, double jmax, double &qcrt, double &vcrt, double &acrt, double &jcrt, size_t &T)->void;
	auto sCurve0Param(double q0, double q1, double vmax, double amax, double jmax, double &taj, double &ta, double &tv, double &tdj, double &td, double &vlim, double &alima, double &alimd)->void;
	auto sCurve0(double t, double q_start, double q_end, double v_max, double a_max, double j_max, double &q_crt, double &v_crt, double &a_crt, double &j_crt, size_t &count)->void;


}



#endif