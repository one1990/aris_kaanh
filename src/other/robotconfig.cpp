#include"robotconfig.h"
#include"stdio.h"
#include<math.h>
#include<array>
#include <aris.hpp>
using namespace CONFIG;
using namespace aris::plan;
using namespace aris::dynamic;




robotconfig::robotconfig()
    {

	     
    }


 void robotJacobian(const double* q, double* JacoEnd)
        {
	 double t1, t2, t3, t4, t5, t6, t7, t8, t9, t10, t11, t12, t13, t14, t15, t16, t17;
	 double t18, t19, t20, t21, t22, t23, t24, t25, t26, t27, t28, t29, t30, t31, t32;
	 double t33, t34, t35, t36, t37, t38, t39, t40, t41, t42, t43, t44, t45, t46, t47;
	 double t48, t49, t50, t51, t52, t53, t54, t55, t56, t57, t58, t59, t60, t61, t62;
	 double t63, t64, t65, t66, t67, t68, t69, t70, t71, t72, t73, t74, t75, t76, t77;
     double q1, q2, q3, q4, q5, q6;
			
            q1 = q[0]  * DirectionFlag[0] + JointOffset[0] + ZeroOffset[0];
            q2 = q[1]  * DirectionFlag[1] + JointOffset[1] + ZeroOffset[1];
            q3 = q[2]  * DirectionFlag[2] + JointOffset[2] + ZeroOffset[2];
            q4 = q[3]  * DirectionFlag[3] + JointOffset[3] + ZeroOffset[3];
            q5 = q[4]  * DirectionFlag[4] + JointOffset[4] + ZeroOffset[4];
            q6 = q[5]  * DirectionFlag[5] + JointOffset[5] + ZeroOffset[5]; 
			
			double MC1, MC2, MC3;
            MC1 = EndPoint[0]; MC2 = EndPoint[1]; MC3 = EndPoint[2];

			double A0[6][6];
			for (int i = 0;i < 6;i++)
				for (int j = 0;j < 6;j++)
					A0[i][j]=0;
            
			t2 = cos(q6);
			t3 = sin(q6);
			t4 = cos(q5);
			t5 = sin(q5);
			t6 = MC1 * t2;
			t14 = MC2 * t3;
			t7 = t6 - t14;
			t8 = sin(q4);
			t9 = MC2 * t2;
			t10 = MC1 * t3;
			t11 = t9 + t10;
			t12 = cos(q4);
			t13 = MC3 * t5;
			t22 = t4 * t7;
			t15 = t13 - t22;
			t16 = cos(q3);
			t17 = MC3 * t4;
			t18 = t5 * t7;
			t19 = d4 + t17 + t18;
			t20 = sin(q3);
			t21 = t8 * t11;
			t23 = t12 * t15;
			t24 = -a4 + t21 + t23;
			t25 = cos(q1);
			t26 = sin(q2);
			t27 = t19 * t20;
			t28 = t16 * t24;
			t29 = -a3 + t27 + t28;
			t30 = cos(q2);
			t31 = t16 * t19;
			t33 = t20 * t24;
			t32 = t31 - t33;
			t34 = sin(q1);
			t35 = t8 * t15;
			t36 = t17 + t18;
			t37 = t7 * t8;
			t38 = t4 * t11*t12;
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
			t50 = cos(t49);
			t52 = t15 * t20;
			t68 = t12 * t16*t36;
			t53 = t52 - t68;
			t54 = t30 * t53;
			t55 = t15 * t16;
			t56 = t12 * t20*t36;
			t57 = t55 + t56;
			t58 = t26 * t57;
			t59 = t54 + t58;
			t60 = t16 * t39;
			t69 = t5 * t11*t20;
			t61 = t60 - t69;
			t62 = t20 * t39;
			t63 = t5 * t11*t16;
			t64 = t62 + t63;
			t65 = t26 * t64;
			t66 = t7 * t12;
			t67 = t66 - t4 * t8*t11;
			t70 = sin(t49);
			A0[0][0] = -t25 * t40 + t34 * t43;
			A0[0][1] = t25 * t45;
			A0[0][2] = -t25 * (t30*t32 - t26 * t47);
			A0[0][3] = -t34 * t48 + t25 * t40*t50;
			A0[0][4] = t25 * t59 - t8 * t34*t36;
			A0[0][5] = t34 * t67 + t25 * (t65 - t30 * t61);
			A0[1][0] = -t25 * t43 - t34 * t40;
			A0[1][1] = t34 * t45;
			A0[1][2] = -t34 * (t46 - t26 * t47);
			A0[1][3] = t25 * t48 + t34 * t50*(t35 - t51);
			A0[1][4] = t34 * t59 + t8 * t25*t36;
			A0[1][5] = -t25 * t67 + t34 * (t65 - t30 * t61);
			A0[2][1] = t41 + t42;
			A0[2][2] = t42 + t30 * t47;
			A0[2][3] = -t40 * t70;
			A0[2][4] = -t26 * t53 + t30 * t57;
			A0[2][5] = t26 * t61 + t30 * t64;
			A0[3][1] = -t34;
			A0[3][2] = -t34;
			A0[3][3] = -t25 * t70;
			A0[3][4] = -t12 * t34 + t8 * t25*t50;
			A0[3][5] = -t5 * (t8*t34 + t12 * t25*t50) - t4 * t25*t70;
			A0[4][1] = t25;
			A0[4][2] = t25;
			A0[4][3] = -t34 * t70;
			A0[4][4] = t12 * t25 + t8 * t34*t50;
			A0[4][5] = t5 * (t8*t25 - t12 * t34*t50) - t4 * t34*t70;
			A0[5][0] = 1.0;
			A0[5][3] = -t50;
			A0[5][4] = -t8 * t70;
			A0[5][5] = -t4 * t50 + t5 * t12*t70;


			for (int i = 0;i < 6;i++)
				for (int j = 0;j < 6;j++)
					JacoEnd[i * 6 + j] = A0[i][j];

 
        }

inline int id(int m, int n, int cols) { return m * cols + n; }

void robotconfig::jointIncrement(const double* q, const double* dX, double* dTheta)
        { 
	       double JacoEnd[36];
	       robotJacobian(q,JacoEnd);

		   
		   // Çó½â AµÄ¹ãÒåÄæpinv ºÍ x
		   double pinv[36];

		   // ËùÐèµÄÖÐ¼ä±äÁ¿£¬Çë¶ÔUµÄ¶Ô½ÇÏßÔªËØ×ö´¦Àí
		   double U[36], tau[6];
		   aris::Size p[6];
		   aris::Size rank;

		   // ¸ù¾Ý A Çó³öÖÐ¼ä±äÁ¿£¬Ïàµ±ÓÚ×ö QR ·Ö½â //
		   // Çë¶Ô U µÄ¶Ô½ÇÏßÔªËØ×ö´¦Àí
		   s_householder_utp(6, 6, JacoEnd, U, tau, p, rank, 1e-10);
           for(int i=0;i<6;i++)
               if(U[7*i]>=0)
                     U[7*i]=U[7*i]+0.1;
               else
                    U[7*i]=U[7*i]-0.1;
		   // ¸ù¾ÝQR·Ö½âµÄ½á¹ûÇóx£¬Ïàµ±ÓÚMatlabÖÐµÄ x = A\b //
		   s_householder_utp_sov(6, 6, 1, rank, U, tau, p, dX, dTheta, 1e-10);

		   // ¸ù¾ÝQR·Ö½âµÄ½á¹ûÇó¹ãÒåÄæ£¬Ïàµ±ÓÚMatlabÖÐµÄ pinv(A) //
		   double tau2[6];
		   s_householder_utp2pinv(6, 6, rank, U, tau, p, pinv, tau2, 1e-10);

		   // ¸ù¾ÝQR·Ö½âµÄ½á¹ûÇó¹ãÒåÄæ£¬Ïàµ±ÓÚMatlabÖÐµÄ pinv(A)*b //
		   s_mm(6, 1, 6, pinv, dX, dTheta);

                
        }
		
		void robotTransform(const double* q, double* TransVector)
        {
            double q1, q2, q3, q4, q5, q6;

            q1 = q[0] * DirectionFlag[0] + JointOffset[0] + ZeroOffset[0];
            q2 = q[1] * DirectionFlag[1] + JointOffset[1] + ZeroOffset[1];
            q3 = q[2] * DirectionFlag[2] + JointOffset[2] + ZeroOffset[2];
            q4 = q[3] * DirectionFlag[3] + JointOffset[3] + ZeroOffset[3];
            q5 = q[4] * DirectionFlag[4] + JointOffset[4] + ZeroOffset[4];
            q6 = q[5] * DirectionFlag[5] + JointOffset[5] + ZeroOffset[5];

			double B0[4][4];
            for (int i = 0; i < 4; i++)
                for (int j = 0; j < 4; j++)
                    B0[i][j] = 0;

			double t1, t2, t3, t4, t5, t6, t7, t8, t9, t10, t11, t12, t13, t14, t15, t16, t17;
			double t18, t19, t20, t21, t22, t23, t24, t25, t26, t27, t28, t29, t30, t31, t32;
			double t33, t34, t35, t36, t37, t38, t39, t40, t41, t42, t43, t44, t45, t46, t47;
			double t48, t49, t50, t51, t52, t53, t54, t55, t56, t57, t58, t59, t60, t61, t62;
			double t63, t64, t65, t66, t67, t68, t69, t70, t71, t72, t73, t74, t75, t76, t77;
			double DF = EndSensor[2];
            t2 = sin(q1);
            t3 = sin(q4);
            t4 = q2 + q3;
            t5 = cos(t4);
            t6 = cos(q1);
            t7 = cos(q4);
            t8 = cos(q6);
            t9 = t2 * t7;
            t10 = t9 - t3 * t5 * t6;
            t11 = sin(q6);
            t12 = cos(q5);
            t13 = t2 * t3;
            t14 = t5 * t6 * t7;
            t15 = t13 + t14;
            t16 = t12 * t15;
            t17 = sin(t4);
            t18 = sin(q5);
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
            t32 = cos(q2);
            t33 = a3 * t32;
            t34 = a2 + t31 + t33 - d4 * t17;
            t35 = t5 * t18;
            t36 = t7 * t12 * t17;
            t37 = t35 + t36;
            t38 = t7 * t17 * t18;
            B0[0][0] = t10 * t11 + t8 * t19;
            B0[0][1] = t8 * t10 - t11 * t19;
            B0[0][2] = -t15 * t18 - t6 * t12 * t17;
            B0[0][3] = -DF * (t15 * t18 + t6 * t12 * t17) + t6 * t34;
            B0[1][0] = -t11 * t22 - t8 * t27;
            B0[1][1] = -t8 * t22 + t11 * t27;
            B0[1][2] = t30;
            B0[1][3] = DF * t30 + t2 * t34;
            B0[2][0] = -t8 * t37 + t3 * t11 * t17;
            B0[2][1] = t11 * t37 + t3 * t8 * t17;
            B0[2][2] = t38 - t5 * t12;
            B0[2][3] = -a4 * t17 - d4 * t5 - a3 * sin(q2) + DF * (t38 - t5 * t12);
            B0[3][3] = 1.0;

			for (int i = 0;i < 4;i++)
				for (int j = 0;j < 4;j++)
					TransVector[i * 4 + j] = B0[i][j];
        }

		/*
		void crossVector(const double* a, const double* b, double* c)
		{
			c[0] = a[1] * b[2] - b[1] * a[2];
			c[1] = -(a[0] * b[2] - b[0] * a[2]);
			c[2] = a[0] * b[1] - b[0] * a[1];
			
		}*/
		void robotconfig::forceTransform(const double* q, const float* FmInEnd, double* FmInWorld)
        {
			double TransVector[16];
            robotTransform(q,TransVector);
			double TransMatrix[4][4];
			for (int i = 0;i < 4;i++)
				for (int j = 0;j < 4;j++)
					TransMatrix[i][j] = TransVector[4*i+j];

			double n[3]={ TransMatrix[0][0], TransMatrix[0][1], TransMatrix[0][2] };
			double o[3]={ TransMatrix[1][0], TransMatrix[1][1], TransMatrix[1][2] };
			double a[3]={ TransMatrix[2][0], TransMatrix[2][1], TransMatrix[2][2] };
			double p[3]={ TransMatrix[0][3], TransMatrix[1][3], TransMatrix[2][3] };

			FmInWorld[0] = n[0] * FmInEnd[0] + n[1] * FmInEnd[1] + n[2] * FmInEnd[2];
			FmInWorld[1] = o[0] * FmInEnd[0] + o[1] * FmInEnd[1] + o[2] * FmInEnd[2];
			FmInWorld[2] = a[0] * FmInEnd[0] + a[1] * FmInEnd[1] + a[2] * FmInEnd[2];

			//std::array<double, 3> crossFP;
           // crossFP = crossVector(f, p);
           // for (int i = 0; i < 3; i++)
             //   crossFP[i] = crossFP[i] + m[i];

			FmInWorld[3] = n[0] * FmInEnd[3] + n[1] * FmInEnd[4] + n[2] * FmInEnd[5];
			FmInWorld[4] = o[0] * FmInEnd[3] + o[1] * FmInEnd[4] + o[2] * FmInEnd[5];
			FmInWorld[5] = a[0] * FmInEnd[3] + a[1] * FmInEnd[4] + a[2] * FmInEnd[5];


        }



