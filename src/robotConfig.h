#ifndef ROBOTCONFIG_H_
#define ROBOTCONFIG_H_

#include <memory>
#include <aris_control.h>
#include <aris_dynamic.h>
#include <aris_plan.h>
#include <atomic>


namespace AcqTor
{
	class Robot
	{
		private double t1, t2, t3, t4, t5, t6, t7, t8, t9, t10, t11, t12, t13, t14, t15, t16, t17,
			t18, t19, t20, t21, t22, t23, t24, t25, t26, t27, t28, t29, t30, t31, t32,
			t33, t34, t35, t36, t37, t38, t39, t40, t41, t42, t43, t44, t45, t46, t47,
			t48, t49, t50, t51, t52, t53, t54, t55, t56, t57, t58, t59, t60, t61, t62,
			t63, t64, t65, t66, t67, t68, t69, t70, t71, t72, t73, t74, t75, t76, t77;
		private double[, ] A0, B0;
		private double[] dTheta;
		private double MC1, MC2, MC3;

		public Robot() {};

		public void robotJacobian(double[] q) {};

		public double[] jointIncrement(double[] q, double[, ] dX) {};


		public double[] crossVector(double[] a, double[] b){};
		public void robotTransform(double[] q){};

		public double[] forceTransform(double[] q, double[] f, double[] m){};
	}
}

#endif