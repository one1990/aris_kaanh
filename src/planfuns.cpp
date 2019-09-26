#include"planfuns.h"
#include"stdio.h"
#include<math.h>
#include<array>
#include <aris.hpp>



namespace traplan
{
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

		T[0] = Tj; T[1] = Ta; T[2] = Tv; T[3] = Tmin;
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


	auto sCurve1(double t, double q_start, double q_end, double v_max, double a_max, double j_max, double &q_crt, double &v_crt, double &a_crt, double &j_crt, size_t &T)->void
	{
		//参数初始化
		//边界参数初始化
		double q0 = q_start;
		double q1 = q_end;
		double v0 = 0;
		double v1 = 0;
		double a0 = 0;
		double a1 = 0;

		//约束参数初始化
		double vmax = v_max;
		double amax = a_max;
		double jmax = j_max;
		double vmin = -v_max;
		double amin = -a_max;
		double jmin = -j_max;

		double t1, t2, t3, t4, t5, t6, t7;//t1-加加速时间段，t2-匀加速时间段，t3-减加速时间段，t4-匀速时间段，t5-加减速时间段，t6-匀减速时间段，t7-减减速时间段
		double ta, tv, td;//ta-加速阶段，tv-匀速阶段，td-减速阶段
		/*
		//判断jerk是否仅有一个双脉冲（一正一负为一个双脉冲）
		bool is_double_dp;
		double t = std::min(std::sqrt(std::abs(v1 - v0) / jmax), amax / jmax);
		if (t == amax / jmax)   //加速度达到最大值amax，判断是否存在jerk等于0的分段
		{
			if (q1 - q0 > 1 / 2 * (v1 + v0) * (amax / jmax + std::abs(v1 - v0) / amax))
			{
				is_double_dp = true;    //存在jerk=0分段

			}
			else
			{
				is_double_dp = false;    //不存在jerk=0分段
			}
		}
		else if (t < amax / jmax)    //加速度未达到最大值amax，判断是否存在jerk等于0的分段
		{
			if (q1 - q0 > t*(v1 + v0))
			{
				is_double_dp = true;    //存在jerk=0分段

			}
			else
			{
				is_double_dp = false;    //不存在jerk=0分段
			}
		}
		*/
		//假设极限速度vlim达到vmax，计算匀速段时间tv
		bool is_to_vmax, vlim, alima, alimd;
		//加速阶段
		if ((vmax - v0) * jmax < amax * amax)
		{
			//极限加速度alim未达到amax
			t2 = 0;
			t1 = t3 = std::sqrt((vmax - v0) / jmax);
			ta = 2 * std::sqrt((vmax - v0) / jmax);
			alima = t1 * jmax;
		}
		else
		{
			//极限加速度alim达到amax
			t1 = t3 = amax / jmax;
			ta = (vmax - v0) / amax + amax / jmax;
			t2 = (vmax - v0) / amax - amax / jmax;
			alima = amax;
		}
		//减速阶段
		if ((vmax - v1) * jmax < amax * amax)
		{
			//极限减速度alim未达到-amax
			t6 = 0;
			t5 = t7 = std::sqrt((vmax - v1) / jmax);
			td = 2 * std::sqrt((vmax - v1) / jmax);
			alimd = t5 * (-jmax);
		}
		else
		{
			//极限减速度alim达到-amax
			t5 = t7 = amax / jmax;
			td = (vmax - v1) / amax + amax / jmax;
			t6 = (vmax - v1) / amax - amax / jmax;
			alimd = -amax;
		}
		//匀速阶段
		tv = (q1 - q0) / vmax - 1 / 2.0 * ta *(1 + v0 / vmax) - 1 / 2.0 * td * (1 + v1 / vmax);
		//判断vlim是否达到最大速度vmax
		if (tv > 0)
		{
			//速度达到vmax
			is_to_vmax = true;
			vlim = vmax;
			t4 = tv;
		}
		else
		{
			//速度未达到最大值vmax
			is_to_vmax = false;
			t4 = tv = 0;
			//假设vlim未达到vmax，则tv=0,计算ta和td
			double delta;
			delta = std::pow(amax, 4) + 2 * (v0 * v0 + v1 * v1) + amax * (4 * (q1 - q0) - 2 * amax * (v0 + v1) / jmax);
			ta = (amax * amax / jmax - 2 * v0 + std::sqrt(delta)) / 2.0 / amax;
			td = (amax * amax / jmax - 2 * v1 + std::sqrt(delta)) / 2.0 / amax;
			//判断ta和td
			if (ta < 0 || td < 0)
			{
				if (ta < 0)
				{
					//不存在加速阶段v0>v1，只存在t5-t7阶段
					ta = 0;
					td = 2 * (q1 - q0) / (v1 + v0);
					t5 = (jmax*(q1 - q0) - std::sqrt(jmax*(jmax*(q1 - q0)*(q1 - q0) + (v1 + v0)*(v1 + v0)*(v1 - v0)))) / jmax / (v1 + v0);
					alimd = -jmax * t5;
					vlim = v1 - (td - t5) * alimd;
					t1 = t2 = t3 = 0;
					t5 = t7 = (td - t6) / 2.0;
				}
				if (td < 0)
				{
					//不存在减速阶段v1>v0,只存在t1-t3阶段
					td = 0;
					ta = 2 * (q1 - q0) / (v1 + v0);
					t1 = (jmax*(q1 - q0) - std::sqrt(jmax*(jmax*(q1 - q0)*(q1 - q0) + (v1 + v0)*(v1 + v0)*(v1 - v0)))) / jmax / (v1 + v0);
					alima = jmax * t1;
					vlim = v0 + (ta - t1)*alima;
					t1 = t3 = (ta - t2) / 2.0;
					t5 = t6 = t7 = 0;
				}
			}
			else
			{
				//判断加速度是否达到最大值
				if (ta > 2 * amax / jmax && td > 2 * amax / jmax)
				{
					//alim达到最大值amax
					t1 = t3 = t5 = t7 = amax / jmax;
					t2 = ta - 2 * t1;
					t6 = td - 2 * t5;
					alima = amax;
					alimd = -amax;
					vlim = v0 + 1 / 2.0 * jmax * t1 * t1 + amax * t2 + 1 / 2.0 * jmax * t3 * t3;
				}
				else
				{
					//alim未达到amax或-amax，需修正amax
					double gama = 0.9;//gama值的范围(0,1)
					while (ta < 2 * amax / jmax || td < 2 * amax / jmax)
					{
						amax = gama * amax;
						delta = std::pow(amax, 4) + 2 * (v0 * v0 + v1 * v1) + amax * (4 * (q1 - q0) - 2 * amax * (v0 + v1) / jmax);
						ta = (amax * amax / jmax - 2 * v0 + std::sqrt(delta)) / 2.0 / amax;
						td = (amax * amax / jmax - 2 * v1 + std::sqrt(delta)) / 2.0 / amax;

					}
					//再次判断ta和td
					if (ta < 0 || td < 0)
					{
						if (ta < 0)
						{
							//不存在加速阶段v0>v1，只存在t5-t7阶段
							ta = 0;
							td = 2 * (q1 - q0) / (v1 + v0);
							t5 = (jmax*(q1 - q0) - std::sqrt(jmax*(jmax*(q1 - q0)*(q1 - q0) + (v1 + v0)*(v1 + v0)*(v1 - v0)))) / jmax / (v1 + v0);
							alimd = -jmax * t5;
							vlim = v1 - (td - t5) * alimd;
							t1 = t2 = t3 = 0;
							t5 = t7 = (td - t6) / 2.0;
						}
						if (td < 0)
						{
							//不存在减速阶段v1>v0,只存在t1-t3阶段
							td = 0;
							ta = 2 * (q1 - q0) / (v1 + v0);
							t1 = (jmax*(q1 - q0) - std::sqrt(jmax*(jmax*(q1 - q0)*(q1 - q0) + (v1 + v0)*(v1 + v0)*(v1 - v0)))) / jmax / (v1 + v0);
							alima = jmax * t1;
							vlim = v0 + (ta - t1)*alima;
							t1 = t3 = (ta - t2) / 2.0;
							t5 = t6 = t7 = 0;
						}
					}
					else
					{
						t1 = t3 = t5 = t7 = amax / jmax;
						t2 = ta - 2 * t1;
						t6 = td - 2 * t5;
						alima = amax;
						alimd = -amax;
						vlim = v0 + 1 / 2.0 * jmax * t1 * t1 + amax * t2 + 1 / 2.0 * jmax*t3*t3;
					}

				}
			}

		}

		//开始计算轨迹
		if (t <= t1)
		{
			q_crt = q0 + v0 * t + jmax * std::pow(t, 3) / 6.0;
			v_crt = v0 + jmax * std::pow(t, 2) / 2.0;
			a_crt = jmax * t;
			j_crt = jmax;
		}
		else if (t <= t1 + t2)
		{
			q_crt = q0 + v0 * t + alima / 6.0 * (3 * t * t - 3 * t1 * t + t1 * t1);
			v_crt = v0 + alima * (t - 1 / 2.0 * t1);
			a_crt = alima;//alima = jmax * t1
			j_crt = 0;
		}
		else if (t <= ta)
		{
			q_crt = q0 + (vlim + v0) * ta / 2.0 - vlim * (ta - t) - jmin * std::pow((ta - t), 3) / 6.0;
			v_crt = vlim + jmin * std::pow(ta - t, 2) / 2.0;
			a_crt = -jmin * (ta - t);
			j_crt = jmin;//jmin=-jmax
		}
		else if (t <= ta + tv)
		{
			q_crt = q0 + (vlim + v0) * ta / 2.0 + vlim * (t - ta);
			v_crt = vlim;
			a_crt = 0;
			j_crt = 0;
		}
		else if (t <= ta + tv + t5)
		{
			q_crt = q1 - (vlim + v1) * td / 2.0 + vlim * (t - ta - tv) - jmax * (std::pow(t - ta - tv, 3)) / 6.0;
			v_crt = vlim - jmax * std::pow(t - ta - tv, 2) / 2.0;
			a_crt = -jmax * (t - ta - tv);
			j_crt = jmin;
		}
		else if (t <= ta + tv + t5 + t6)
		{
			q_crt = q1 - (vlim + v1) * td / 2.0 + vlim * (t - ta - tv) + alimd / 6.0 * (3 * std::pow(t - ta - tv, 2) - 3 * t5 * (t - ta - tv) + t5 * t5);
			v_crt = vlim + alimd * (t - ta - tv - t5 / 2.0);
			a_crt = alimd;//alimd = -jmax * t5
			j_crt = 0;
		}
		else if (t <= ta + tv + td)
		{
			q_crt = q1 - v1 * (ta + tv + td - t) - jmax / 6 * std::pow(ta + tv + td - t, 3);
			v_crt = v1 + jmax / 2.0 * std::pow(ta + tv + td - t, 2);
			a_crt = -jmax * (ta + tv + td - t);
			j_crt = jmax;
		}

	}

	auto sCurveParam(double q0, double q1, double vmax, double amax, double jmax, double &taj, double &ta, double &tv, double &tdj, double &td, double &vlim, double &alim, double &jlim)->void
	{
		//加、减速对称情况下，轨迹总是存在的
		//double t1, t2, t3, t4, t5, t6, t7;//t1-加加速时间段，t2-匀加速时间段，t3-减加速时间段，t4-匀速时间段，t5-加减速时间段，t6-匀减速时间段，t7-减减速时间段
		//计算tv,判断速度vlim是否达到最大值
		double alima, alimd;
		size_t Taj, Ta, Tv;
		//加速阶段、减速阶段
		if (vmax * jmax >= amax * amax)
		{
			//加速度alim达到amax或-amax
			taj = amax / jmax;
			ta = vmax / amax + taj;
			Taj = (taj - std::floor(taj) <=1e-3) ? taj : static_cast<size_t>(std::ceil(taj));
			//std::cout << taj << std::endl;
			alim = amax;
			Ta = (ta - std::floor(ta) <= 1e-3) ? ta : static_cast<size_t>(std::ceil(ta));
			vlim = alim * (Ta - Taj);
		}
		else
		{
			//加速度alim未达到amax或-amax
			taj = std::sqrt(vmax / jmax);
			ta = 2.0 * taj;
			Taj = (taj - std::floor(taj) <= 1e-3) ? taj : static_cast<size_t>(std::ceil(taj));
			alim = jmax * Taj;
			Ta = static_cast<size_t>(2.0 * Taj);
			vlim = alim * (Ta - Taj);
		}

		//匀速阶段
		tv = std::abs(q1 - q0) / vmax - ta;
		//double sa, sv, sd;
		//sa = vmax * Ta / 2;
		//sd = sa;
		//sv = q1 - q0 - sa - sd;
		//Tv = sv / vmax;
		Tv = (tv - std::floor(tv) <= 1e-3) ? tv : static_cast<size_t>(std::ceil(tv));
		
		//判断vlim是否达到最大速度vmax
		if (tv > 0)
		{
			//按比例修正约束条件
			double k;
			k = std::abs(q1 - q0) / ((Tv + Ta) * vlim);
			vlim *= k;
			alim *= k;
			jlim = jmax * k;
			alima = alim;
			alimd = -alim;
			
		}
		else
		{
			Tv = 0;
			//vlim < vmax时，判断加速度是否到达最大
			if ((q1 - q0) >= 2.0 * std::pow(amax, 3.0) / jmax / jmax)
			{
				//加速度alim达到最大值amax或-amax
				taj = amax / jmax;
				ta = taj / 2.0 + std::sqrt(std::pow(taj / 2.0, 2.0) + (q1 - q0) / amax);
				Taj = (taj - std::floor(taj) <= 1e-3) ? taj : static_cast<size_t>(std::ceil(taj));
				alim = amax;
				Ta = (ta - std::floor(ta) <= 1e-3) ? ta : static_cast<size_t>(std::ceil(ta));
				vlim = (Ta - Taj) * alim;
			}
			else
			{
				//加速度alim未达到最大值amax或-amax
				taj = std::pow((q1 - q0) / 2.0 / jmax, 1.0 / 3.0);
				//ta = 2.0 * taj;
				Taj = (taj - std::floor(taj) <= 1e-3) ? taj : static_cast<size_t>(std::ceil(taj));
				alim = jmax * Taj;
				Ta = static_cast<size_t>(2.0 * Taj);
				vlim = (Ta - Taj) * alim;
			}
			//按比例修正约束条件
			double k;
			k = std::abs(q1 - q0) / (Ta * vlim);
			vlim *= k;
			alim *= k;
			jlim = jmax * k;
			alima = alim;
			alimd = -alim;
		}
		td = ta = (double)Ta;
		tdj = taj = (double)Taj;
		tv = (double)Tv;
		/*
		std::cout << std::setiosflags(std::ios::fixed);
		std::cout << std::setprecision(4);
		std::cout << "taj=" << taj << "  ";
		std::cout << "ta=" << ta << "  ";
		std::cout << "tv=" << tv << "  ";
		std::cout << "tdj=" << tdj << "  ";
		std::cout << "td=" << td << "  ";
		std::cout << "t=" << ta + tv + td << "  ";
		std::cout << "vlim=" << vlim * 1000.0 << "  ";
		std::cout << "alim=" << alim * 1000.0 * 1000.0 << "  ";
		std::cout << "jlim=" << jlim * 1000.0 * 1000.0 * 1000.0 << "  ";
		std::cout << std::endl;
		*/
	}
	auto sCurve(size_t t, double q_start, double q_end, double v_max, double a_max, double j_max, double &q_crt, double &v_crt, double &a_crt, double &j_crt, size_t &T)->void
	{
		//参数初始化
		//边界参数初始化
		double q0 = 0;
		double q1 = 0;
		double v0 = 0;
		double v1 = 0;
		double a0 = 0;
		double a1 = 0;
		//约束参数初始化
		double vmax = 0;
		double amax = 0;
		double jmax = 0;
		double vmin = -0;
		double amin = -0;
		double jmin = -0;
		//比较q_start和q_end，修改约束条件
		double sigma;
		if (q_start <= q_end)
		{
			sigma = 1;
		}
		else if (q_start > q_end)
		{
			sigma = -1;
		}
		q0 = sigma * q_start;
		q1 = sigma * q_end;
		vmax = (sigma + 1) / 2.0 * std::abs(v_max) + (sigma - 1) / 2.0 * -std::abs(v_max);
		vmin = (sigma + 1) / 2.0 * -std::abs(v_max) + (sigma - 1) / 2.0 * std::abs(v_max);
		amax = (sigma + 1) / 2.0 * std::abs(a_max) + (sigma - 1) / 2.0 * -std::abs(a_max);
		amin = (sigma + 1) / 2.0 * -std::abs(a_max) + (sigma - 1) / 2.0 * std::abs(a_max);
		jmax = (sigma + 1) / 2.0 * std::abs(j_max) + (sigma - 1) / 2.0 * -std::abs(j_max);
		jmin = (sigma + 1) / 2.0 * -std::abs(j_max) + (sigma - 1) / 2.0 * std::abs(j_max);

		//时间离散化，最小单位时间1ms
		double taj, ta, tv, tdj, td, vlim, alim, jlim;
		sCurveParam(q0, q1, vmax, amax, jmax, taj, ta, tv, tdj, td, vlim, alim, jlim);
		double alima, alimd, jlima, jlimd;
		alima = alim;
		alimd = -alim;
		jlima = jlim;
		jlimd = -jlim;
		T = ta + tv + td;
		//开始计算轨迹，时间t的单位是ms
		if (t <= taj)
		{
			q_crt = q0 + v0 * t + jlima * std::pow(t, 3) / 6.0;
			v_crt = v0 + jlima * std::pow(t, 2) / 2.0;
			a_crt = jlima * t;
			j_crt = jlima;
		}
		else if (t <= ta - taj)
		{
			q_crt = q0 + v0 * t + alima / 6.0 * (3 * t * t - 3 * taj * t + taj * taj);
			v_crt = v0 + alima * (t - taj / 2.0);
			a_crt = alima;
			j_crt = 0;
		}
		else if (t <= ta)
		{
			q_crt = q0 + (vlim + v0) * ta / 2.0 - vlim * (ta - t) - jlimd * std::pow(ta - t, 3) / 6.0;
			v_crt = vlim + jlimd * std::pow(ta - t, 2) / 2.0;
			a_crt = -jlimd * (ta - t);
			j_crt = jlimd;
		}
		else if (t <= ta + tv)
		{
			q_crt = q0 + (vlim + v0) * ta / 2.0 + vlim * (t - ta);
			v_crt = vlim;
			a_crt = 0;
			j_crt = 0;
		}
		else if (t <= ta + tv + tdj)
		{
			q_crt = q1 - (vlim + v1) * td / 2.0 + vlim * (t - ta - tv) - jlima * (std::pow(t - ta - tv, 3)) / 6.0;
			v_crt = vlim - jlima * std::pow(t - ta - tv, 2) / 2.0;
			a_crt = -jlima * (t - ta - tv);
			j_crt = -jlima;
		}
		else if (t <= ta + tv + td - tdj)
		{
			q_crt = q1 - (vlim + v1) * td / 2.0 + vlim * (t - ta - tv) + alimd / 6.0 * (3 * std::pow(t - ta - tv, 2) - 3 * tdj * (t - ta - tv) + tdj * tdj);
			v_crt = vlim + alimd * (t - ta - tv - tdj / 2.0);
			a_crt = alimd;
			j_crt = 0;
		}
		else if (t <= ta + tv + td)
		{
			q_crt = q1 - v1 * (ta + tv + td - t) - jlima / 6.0 * std::pow(ta + tv + td - t, 3);
			v_crt = v1 + jlima / 2.0 * std::pow(ta + tv + td - t, 2);
			a_crt = -jlima * (ta + tv + td - t);
			j_crt = jlima;
		}
		//修正位置、速度、加速度、jerk方向
		q_crt = sigma * q_crt;
		v_crt = sigma * v_crt;
		a_crt = sigma * a_crt;
		j_crt = sigma * j_crt;

		/*
		std::cout << t << ",";
		std::cout << q_crt << ",";
		std::cout << v_crt * 1000 << ",";
		std::cout << a_crt * 1000 * 1000 << ",";
		std::cout << j_crt * 1000 * 1000 * 1000;
		std::cout << std::endl;
		*/
	}


	auto sCurve0Param(double q0, double q1, double vmax, double amax, double jmax, double &taj, double &ta, double &tv, double &tdj, double &td, double &vlim, double &alima, double &alimd)->void
	{
		//加、减速对称情况下，轨迹总是存在的
		//double t1, t2, t3, t4, t5, t6, t7;//t1-加加速时间段，t2-匀加速时间段，t3-减加速时间段，t4-匀速时间段，t5-加减速时间段，t6-匀减速时间段，t7-减减速时间段
		//计算tv,判断速度vlim是否达到最大值
		//double vlim, alima, alimd;
		//加速阶段、减速阶段
		if (vmax * jmax >= amax * amax)
		{
			//加速度alim达到amax或-amax
			taj = (double) amax / jmax;
			ta = (double) vmax / amax + taj;
		}
		else
		{
			//加速度alim未达到amax或-amax
			taj = (double) std::sqrt(vmax / jmax);
			ta = (double) 2.0 * taj;
		}

		//匀速阶段
		tv = (double) (q1 - q0) / vmax - ta;

		//判断vlim是否达到最大速度vmax
		if (tv > 0)
		{
			//速度达到vmax
			vlim = vmax;
			alima = jmax * taj;
			alimd = -alima;
		}
		else
		{
			tv = 0;
			//vlim < vmax时，判断加速度是否到达最大
			if (q1 - q0 >= 2.0 * std::pow(amax, 3.0) / jmax / jmax)
			{
				//加速度alim达到最大值amax或-amax
				taj = (double) amax / jmax;
				ta = (double) taj / 2 + std::sqrt(std::pow(taj / 2.0, 2.0) + (q1 - q0) / amax);
			}
			else
			{
				//加速度alim未达到最大值amax或-amax
				taj =  std::pow((q1 - q0) / 2.0 / jmax, 1.0 / 3.0);
				ta = (double) 2.0 * taj;
				
			}
			alima = jmax * taj;
			alimd = -alima;
			vlim = (ta - taj) * alima;
		}
		td = ta;
		tdj = taj;
		//t1 = t3 = taj;
		//t2 = ta - 2 * taj;
		//t4 = tv;
		//t5 = t7 = tdj;
		//t6 = td - 2 * tdj;
		/*
		std::cout << std::setiosflags(std::ios::fixed);
		std::cout << std::setprecision(5);
		std::cout << "taj=" << taj << "  ";
		std::cout << "ta=" << ta << "  ";
		std::cout << "tv=" << tv << "  ";
		std::cout << "tdj=" << tdj << "  ";
		std::cout << "td=" << td << "  ";
		std::cout << "t=" << ta + tv + td << "  ";
		std::cout << "vlim=" << vlim << "  ";
		std::cout << "alima=" << alima << "  ";
		std::cout << "alimd=" << alimd << "  ";
		std::cout << std::endl;*/
	}
	auto sCurve0(double t, double q_start, double q_end, double v_max, double a_max, double j_max, double &q_crt, double &v_crt, double &a_crt, double &j_crt, size_t &count)->void
	{
		
		//参数初始化
		//边界参数初始化
		double q0 = 0;
		double q1 = 0;
		double v0 = 0;
		double v1 = 0;
		double a0 = 0;
		double a1 = 0;
		//约束参数初始化
		double vmax = 0;
		double amax = 0;
		double jmax = 0;
		double vmin = -0;
		double amin = -0;
		double jmin = -0;
		//比较q_start和q_end，修改约束条件
		double sigma;
		if (q_start < q_end)
		{
			sigma = 1;
		}
		else if (q_start > q_end)
		{
			sigma = -1;
		}
		q0 = sigma * q_start;
		q1 = sigma * q_end;
		vmax = (sigma + 1) / 2.0 * std::abs(v_max) + (sigma - 1) / 2.0 * -std::abs(v_max);
		vmin = (sigma + 1) / 2.0 * -std::abs(v_max) + (sigma - 1) / 2.0 * std::abs(v_max);
		amax = (sigma + 1) / 2.0 * std::abs(a_max) + (sigma - 1) / 2.0 * -std::abs(a_max);
		amin = (sigma + 1) / 2.0 * -std::abs(a_max) + (sigma - 1) / 2.0 * std::abs(a_max);
		jmax = (sigma + 1) / 2.0 * std::abs(j_max) + (sigma - 1) / 2.0 * -std::abs(j_max);
		jmin = (sigma + 1) / 2.0 * -std::abs(j_max) + (sigma - 1) / 2.0 * std::abs(j_max);

		//时间离散化，最小单位时间1ms
		double taj, ta, tv, tdj, td, vlim, alima, alimd;
		sCurve0Param(q0, q1, vmax, amax, jmax, taj, ta, tv, tdj, td, vlim, alima, alimd);
		double t1, t2, t3, t4;
		t1 = t3 = taj;
		t2 = ta - 2 * taj;
		t4 = tv;
		double T1, T2, T3, T4;
		double T = 0;
		T1 = std::ceil(t1 * 1000) / 1000.0;
		T2 = std::ceil(t2 * 1000) / 1000.0;
		T3 = std::ceil(t3 * 1000) / 1000.0;
		T4 = std::ceil(t4 * 1000) / 1000.0;
		
		T = (T1 + T2 + T3) * 2 + T4;
		double k = (ta + tv + td) / T;
		
		vmax = vmax * k;
		amax = amax * k * k;
		jmax = jmax * k * k * k;
		sCurve0Param(q0, q1, vmax, amax, jmax, taj, ta, tv, tdj, td, vlim, alima, alimd);
		
		
		count = T * 1000;
		//开始计算轨迹
		if (t <= taj)
		{
			q_crt = q0 + v0 * t + jmax * std::pow(t, 3) / 6.0;
			v_crt = v0 + jmax * std::pow(t, 2) / 2.0;
			a_crt = jmax * t;
			j_crt = jmax;
		}
		else if (t <= ta - taj)
		{
			q_crt = q0 + v0 * t + alima / 6.0 * (3 * t * t - 3 * taj * t + taj * taj);
			v_crt = v0 + alima * (t - taj / 2.0);
			a_crt = alima;
			j_crt = 0;
		}
		else if (t <= ta)
		{
			q_crt = q0 + (vlim + v0) * ta / 2.0 - vlim * (ta - t) - jmin * std::pow(ta - t, 3) / 6.0;
			v_crt = vlim + jmin * std::pow(ta - t, 2) / 2.0;
			a_crt = -jmin * (ta - t);
			j_crt = jmin;//jmin=-jmax
		}
		else if (t <= ta + tv)
		{
			q_crt = q0 + (vlim + v0) * ta / 2.0 + vlim * (t - ta);
			v_crt = vlim;
			a_crt = 0;
			j_crt = 0;
		}
		else if (t <= ta + tv + tdj)
		{
			q_crt = q1 - (vlim + v1) * td / 2.0 + vlim * (t - ta - tv) - jmax * (std::pow(t - ta - tv, 3)) / 6.0;
			v_crt = vlim - jmax * std::pow(t - ta - tv, 2) / 2.0;
			a_crt = -jmax * (t - ta - tv);
			j_crt = jmin;
		}
		else if (t <= ta + tv + td - tdj)
		{
			q_crt = q1 - (vlim + v1) * td / 2.0 + vlim * (t - ta - tv) + alimd / 6.0 * (3 * std::pow(t - ta - tv, 2) - 3 * tdj * (t - ta - tv) + tdj * tdj);
			v_crt = vlim + alimd * (t - ta - tv - tdj / 2.0);
			a_crt = alimd;//alimd = -jmax * tdj
			j_crt = 0;
		}
		else if (t <= ta + tv + td)
		{
			q_crt = q1 - v1 * (ta + tv + td - t) - jmax / 6.0 * std::pow(ta + tv + td - t, 3);
			v_crt = v1 + jmax / 2.0 * std::pow(ta + tv + td - t, 2);
			a_crt = -jmax * (ta + tv + td - t);
			j_crt = jmax;
		}
		//修正位置、速度、加速度、jerk方向
		q_crt = sigma * q_crt;
		v_crt = sigma * v_crt;
		a_crt = sigma * a_crt;
		j_crt = sigma * j_crt;
		

		std::cout << t << ",";
		std::cout << q_crt << ",";
		std::cout << v_crt << ",";
		std::cout << a_crt << ",";
		std::cout << j_crt;
		std::cout << std::endl;
		
	}






}





