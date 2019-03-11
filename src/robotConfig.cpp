#include"robotConfig.h"

using namespace AcqTor;


Robot::Robot()
{
	A0 = new double[common.RobotAxis, common.RobotAxis];
	B0 = new double[4, 4];
	dTheta = new double[6];
}

void Robot::RrobotJacobian(double[] q)
{

	double q1, q2, q3, q4, q5, q6;

	q1 = q[0] * common.ConRad * common.DirectionFlag[0] + common.JointOffset[0] + common.ZeroOffset[0];
	q2 = q[1] * common.ConRad * common.DirectionFlag[1] + common.JointOffset[1] + common.ZeroOffset[1];
	q3 = q[2] * common.ConRad * common.DirectionFlag[2] + common.JointOffset[2] + common.ZeroOffset[2];
	q4 = q[3] * common.ConRad * common.DirectionFlag[3] + common.JointOffset[3] + common.ZeroOffset[3];
	q5 = q[4] * common.ConRad * common.DirectionFlag[4] + common.JointOffset[4] + common.ZeroOffset[4];
	q6 = q[5] * common.ConRad * common.DirectionFlag[5] + common.JointOffset[5] + common.ZeroOffset[5];

	MC1 = common.EndPoint[0]; MC2 = common.EndPoint[1]; MC3 = common.EndPoint[2];

	for (int i = 0; i < common.RobotAxis; i++)
		for (int j = 0; j < common.RobotAxis; j++)
			A0[i, j] = 0;

	#region Jacobain
		double a2, a3, a4, d4;
	a2 = Form2.DHparas[0] / 1000.0;
	a3 = Form2.DHparas[1] / 1000.0;
	a4 = Form2.DHparas[2] / 1000.0;
	d4 = Form2.DHparas[3] / 1000.0;
	t2 = Math.Cos(q6);
	t3 = Math.Sin(q6);
	t4 = Math.Cos(q5);
	t5 = Math.Sin(q5);
	t6 = MC1 * t2;
	t14 = MC2 * t3;
	t7 = t6 - t14;
	t8 = Math.Sin(q4);
	t9 = MC2 * t2;
	t10 = MC1 * t3;
	t11 = t9 + t10;
	t12 = Math.Cos(q4);
	t13 = MC3 * t5;
	t22 = t4 * t7;
	t15 = t13 - t22;
	t16 = Math.Cos(q3);
	t17 = MC3 * t4;
	t18 = t5 * t7;
	t19 = d4 + t17 + t18;
	t20 = Math.Sin(q3);
	t21 = t8 * t11;
	t23 = t12 * t15;
	t24 = -a4 + t21 + t23;
	t25 = Math.Cos(q1);
	t26 = Math.Sin(q2);
	t27 = t19 * t20;
	t28 = t16 * t24;
	t29 = -a3 + t27 + t28;
	t30 = Math.Cos(q2);
	t31 = t16 * t19;
	t33 = t20 * t24;
	t32 = t31 - t33;
	t34 = Math.Sin(q1);
	t35 = t8 * t15;
	t36 = t17 + t18;
	t37 = t7 * t8;
	t38 = t4 * t11 * t12;
	t39 = t37 + t38;
	t51 = t11 * t12;
	t40 = t35 - t51;
	t41 = t29 * t30;
	t42 = t26 * t32;
	t43 = -a2 + t41 + t42;
	t44 = t26 * t29;
	t46 = t30 * t32;
	t45 = t44 - t46;
	t47 = t27 + t28;
	t48 = t21 + t23;
	t49 = q2 + q3;
	t50 = Math.Cos(t49);
	t52 = t15 * t20;
	t68 = t12 * t16 * t36;
	t53 = t52 - t68;
	t54 = t30 * t53;
	t55 = t15 * t16;
	t56 = t12 * t20 * t36;
	t57 = t55 + t56;
	t58 = t26 * t57;
	t59 = t54 + t58;
	t60 = t16 * t39;
	t69 = t5 * t11 * t20;
	t61 = t60 - t69;
	t62 = t20 * t39;
	t63 = t5 * t11 * t16;
	t64 = t62 + t63;
	t65 = t26 * t64;
	t66 = t7 * t12;
	t67 = t66 - t4 * t8 * t11;
	t70 = Math.Sin(t49);
	A0[0, 0] = -t25 * t40 + t34 * t43;
	A0[0, 1] = t25 * t45;
	A0[0, 2] = -t25 * (t30 * t32 - t26 * t47);
	A0[0, 3] = -t34 * t48 + t25 * t40 * t50;
	A0[0, 4] = t25 * t59 - t8 * t34 * t36;
	A0[0, 5] = t34 * t67 + t25 * (t65 - t30 * t61);
	A0[1, 0] = -t25 * t43 - t34 * t40;
	A0[1, 1] = t34 * t45;
	A0[1, 2] = -t34 * (t46 - t26 * t47);
	A0[1, 3] = t25 * t48 + t34 * t50 * (t35 - t51);
	A0[1, 4] = t34 * t59 + t8 * t25 * t36;
	A0[1, 5] = -t25 * t67 + t34 * (t65 - t30 * t61);
	A0[2, 1] = t41 + t42;
	A0[2, 2] = t42 + t30 * t47;
	A0[2, 3] = -t40 * t70;
	A0[2, 4] = -t26 * t53 + t30 * t57;
	A0[2, 5] = t26 * t61 + t30 * t64;
	A0[3, 1] = -t34;
	A0[3, 2] = -t34;
	A0[3, 3] = -t25 * t70;
	A0[3, 4] = -t12 * t34 + t8 * t25 * t50;
	A0[3, 5] = -t5 * (t8 * t34 + t12 * t25 * t50) - t4 * t25 * t70;
	A0[4, 1] = t25;
	A0[4, 2] = t25;
	A0[4, 3] = -t34 * t70;
	A0[4, 4] = t12 * t25 + t8 * t34 * t50;
	A0[4, 5] = t5 * (t8 * t25 - t12 * t34 * t50) - t4 * t34 * t70;
	A0[5, 0] = 1.0;
	A0[5, 3] = -t50;
	A0[5, 4] = -t8 * t70;
	A0[5, 5] = -t4 * t50 + t5 * t12 * t70;


	#endregion
}

double Robot::jointIncrement(double[] q, double[, ] dX)
{
	robotJacobian(q);

	Matrix<double> Jacobian = DenseMatrix.OfArray(A0);
	Svd<Double> Jsvd = Jacobian.Svd(true);
	Matrix<double> U = Jsvd.U;
	Matrix<double> VT = Jsvd.VT;
	Vector<double> Sdiag = Jsvd.S;
	double[] S_diag = new double[6];
	for (int i = 0; i < 6; i++)
		S_diag[i] = Jsvd.S[i];


	//Matrix<double> S = DenseMatrix.OfArray(Sx);
	//Matrix<double> J = U * S * VT;

	double[] SdiagInv = new double[6];
	bool SigularityPoint = false;
	for (int i = 0; i < common.RobotAxis; i++)
	{
		if (Sdiag[i] < 1E-3)
			SigularityPoint = true;

	}
	SigularityPoint = true;
	if (SigularityPoint)
	{
		for (int i = 0; i < common.RobotAxis; i++)
		{
			SdiagInv[i] = S_diag[i] / (S_diag[i] * S_diag[i] + 0.001);
		}
	}
	else
	{
		for (int i = 0; i < common.RobotAxis; i++)
		{
			SdiagInv[i] = 1 / (Sdiag[i]);
		}
	}

	Matrix<double> Sinv = DenseMatrix.OfArray(new double[, ]{
		{SdiagInv[0],0,0,0,0,0},
		{0,SdiagInv[1],0,0,0,0},
		{0,0,SdiagInv[2],0,0,0},
		{0,0,0,SdiagInv[3],0,0},
		{0,0,0,0,SdiagInv[4],0},
		{0,0,0,0,0,SdiagInv[5]} });

	Matrix<double> pinvJ = VT.Transpose() * Sinv * U.Transpose();
	Matrix<double> pJ = Jacobian.PseudoInverse();
	Matrix<double> dXM = DenseMatrix.OfArray(dX);//dX:6*1;
	Matrix<double> dTM = pinvJ * dXM;

	for (int i = 0; i < common.RobotAxis; i++)
		dTheta[i] = dTM[i, 0];


	return dTheta;
}


double Robot::crossVector(double[] a, double[] b)
{
	double[] c = new double[3];
	c[0] = a[1] * b[2] - b[1] * a[2];
	c[1] = -(a[0] * b[2] - b[0] * a[2]);
	c[2] = a[0] * b[1] - b[0] * a[1];
	return c;
}
void Robot::robotTransform(double[] q)
{
	double q1, q2, q3, q4, q5, q6;

	q1 = q[0] * common.ConRad * common.DirectionFlag[0] + common.JointOffset[0] + common.ZeroOffset[0];
	q2 = q[1] * common.ConRad * common.DirectionFlag[1] + common.JointOffset[1] + common.ZeroOffset[1];
	q3 = q[2] * common.ConRad * common.DirectionFlag[2] + common.JointOffset[2] + common.ZeroOffset[2];
	q4 = q[3] * common.ConRad * common.DirectionFlag[3] + common.JointOffset[3] + common.ZeroOffset[3];
	q5 = q[4] * common.ConRad * common.DirectionFlag[4] + common.JointOffset[4] + common.ZeroOffset[4];
	q6 = q[5] * common.ConRad * common.DirectionFlag[5] + common.JointOffset[5] + common.ZeroOffset[5];

	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 4; j++)
			B0[i, j] = 0;

	#region TransformMatrix
		double a2, a3, a4, d4, DF;
	a2 = Form2.DHparas[0] / 1000.0;
	a3 = Form2.DHparas[1] / 1000.0;
	a4 = Form2.DHparas[2] / 1000.0;
	d4 = Form2.DHparas[3] / 1000.0;
	DF = common.EndPoint[2];

	t2 = Math.Sin(q1);
	t3 = Math.Sin(q4);
	t4 = q2 + q3;
	t5 = Math.Cos(t4);
	t6 = Math.Cos(q1);
	t7 = Math.Cos(q4);
	t8 = Math.Cos(q6);
	t9 = t2 * t7;
	t10 = t9 - t3 * t5 * t6;
	t11 = Math.Sin(q6);
	t12 = Math.Cos(q5);
	t13 = t2 * t3;
	t14 = t5 * t6 * t7;
	t15 = t13 + t14;
	t16 = t12 * t15;
	t17 = Math.Sin(t4);
	t18 = Math.Sin(q5);
	t19 = t16 - t6 * t17 * t18;
	t20 = t6 * t7;
	t21 = t2 * t3 * t5;
	t22 = t20 + t21;
	t23 = t3 * t6;
	t28 = t2 * t5 * t7;
	t24 = t23 - t28;
	t25 = t12 * t24;
	t26 = t2 * t17 * t18;
	t27 = t25 + t26;
	t29 = t18 * t24;
	t30 = t29 - t2 * t12 * t17;
	t31 = a4 * t5;
	t32 = Math.Cos(q2);
	t33 = a3 * t32;
	t34 = a2 + t31 + t33 - d4 * t17;
	t35 = t5 * t18;
	t36 = t7 * t12 * t17;
	t37 = t35 + t36;
	t38 = t7 * t17 * t18;
	B0[0, 0] = t10 * t11 + t8 * t19;
	B0[0, 1] = t8 * t10 - t11 * t19;
	B0[0, 2] = -t15 * t18 - t6 * t12 * t17;
	B0[0, 3] = -DF * (t15 * t18 + t6 * t12 * t17) + t6 * t34;
	B0[1, 0] = -t11 * t22 - t8 * t27;
	B0[1, 1] = -t8 * t22 + t11 * t27;
	B0[1, 2] = t30;
	B0[1, 3] = DF * t30 + t2 * t34;
	B0[2, 0] = -t8 * t37 + t3 * t11 * t17;
	B0[2, 1] = t11 * t37 + t3 * t8 * t17;
	B0[2, 2] = t38 - t5 * t12;
	B0[2, 3] = -a4 * t17 - d4 * t5 - a3 * Math.Sin(q2) + DF * (t38 - t5 * t12);
	B0[3, 3] = 1.0;


	#endregion



}

double Robot::forceTransform(double[] q, double[] f, double[] m)
{
	robotTransform(q);
	double[] NewFrameF = new double[6];

	double[] n = new double[3]{ B0[0, 0], B0[0, 1], B0[0, 2] };
	double[] o = new double[3]{ B0[1, 0], B0[1, 1], B0[1, 2] };
	double[] a = new double[3]{ B0[2, 0], B0[2, 1], B0[2, 2] };
	double[] p = new double[3]{ B0[0, 3], B0[1, 3], B0[2, 3] };

	NewFrameF[0] = n[0] * f[0] + n[1] * f[1] + n[2] * f[2];
	NewFrameF[1] = o[0] * f[0] + o[1] * f[1] + o[2] * f[2];
	NewFrameF[2] = a[0] * f[0] + a[1] * f[1] + a[2] * f[2];

	double[] crossFP = new double[3];
	crossFP = crossVector(f, p);
	for (int i = 0; i < 3; i++)
		crossFP[i] = crossFP[i] + m[i];

	NewFrameF[3] = n[0] * crossFP[0] + n[1] * crossFP[1] + n[2] * crossFP[2];
	NewFrameF[4] = o[0] * crossFP[0] + o[1] * crossFP[1] + o[2] * crossFP[2];
	NewFrameF[5] = a[0] * crossFP[0] + a[1] * crossFP[1] + a[2] * crossFP[2];

	return NewFrameF;

}

