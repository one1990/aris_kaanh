#include"planfuns.h"
#include"stdio.h"
#include<math.h>
#include<array>
#include <aris.hpp>
using namespace PLANGK;
using namespace aris::plan;
using namespace aris::dynamic;


planconfig::planconfig()
{


}

void planconfig::DoubleSVelocity(const double jMax, const double aMax, const double vMax, const double sMax, double *T, double &vlim, double &alim)
{
	double Tj = 0, Ta = 0, Tv = 0;
	if (vMax*jMax >= aMax * aMax)
	{
		Tj = aMax / jMax;
		Ta = Tj + vMax / aMax;
	}
	else
	{
		Tj = sqrt(vMax / jMax);
		Ta = 2 * Tj;
	}

	Tv = sMax / vMax - Ta;
	if (Tv <= 0)
	{
		Tv = 0;
		if (sMax >= 2 * aMax*aMax*aMax / jMax / jMax)
		{
			Tj = aMax / jMax;
			Ta = Tj / 2 + sqrt((Tj / 2)*(Tj / 2) + sMax / aMax);
		}
		else
		{
			Tj = pow(sMax / jMax / 2, 1 / 3);
			Ta = 2 * Tj;
		}
	}

	alim = jMax * Tj;
	vlim = (Ta - Tj)*alim;

	double Tmin = 2 * Ta + Tv;

	T[0] = Tj;T[1] = Ta;T[2] = Tv; T[3] = Tmin;
}
void planconfig::RealSVelocity(const double t, const double jMax, const double aMax, const double vMax, const double sMax, const double* Tt, const double vlim, const double alim, double &st, double &vt, double &at, double &jt)
{
	double Tj = Tt[0], Ta = Tt[1], Tv = Tt[2], T = Tt[3];//T[0]:Tj;  T[1]:Ta;  T[2]:Tv;  T[3]:Tmin
	if (t < Tj)
	{
		st = jMax * t*t*t / 6;
		vt = jMax * t*t / 2;
		at = jMax * t;
		jt = jMax;
	}
	else if (t <= Ta - Tj)
	{
		st = alim / 6 * (3 * t*t - 3 * Tj*t + Tj * Tj);
		vt = alim * (t - Tj / 2);
		at = alim;
		jt = 0;
	}
	else if (t <= Ta)
	{
		st = vlim * Ta / 2 - vlim * (Ta - t) + jMax * (Ta - t)*(Ta - t)*(Ta - t) / 6;
		vt = vlim - jMax * (Ta - t)*(Ta - t) / 2;
		at = jMax * (Ta - t);
		jt = -jMax;
		
	}
	else if (t <= Ta + Tv)
	{
		st = vlim * Ta / 2 + vlim * (t - Ta);
		vt = vlim;
		at = 0;
		jt = 0;
	}
	else if (t <= Ta + Tv + Tj)
	{
		st = sMax - vlim * Ta / 2 + vlim * (t - T + Ta) - jMax * (t - T + Ta)*(t - T + Ta)*(t - T + Ta) / 6;
		vt = vlim - jMax * (t - T + Ta)*(t - T + Ta) / 2;
		at = -jMax * (t - T + Ta);
		jt = -jMax;
	}
	else if (t <= T - Tj)
	{
		st = sMax - vlim * Ta / 2 + vlim * (t - T + Ta) - alim / 6 * (3 * (t - T + Ta)*(t - T + Ta) - 3 * Tj*(t - T + Ta) + Tj * Tj);
		vt = vlim - alim * (t - T + Ta - Tj / 2);
		at = -alim;
		jt = 0;
	}
	else
	{
		st = sMax - jMax * (T - t)*(T - t)*(T - t) / 6;
		vt = jMax * (T - t)*(T - t) / 2;
		at = -jMax * (T - t);
		jt = jMax;
	}

}





