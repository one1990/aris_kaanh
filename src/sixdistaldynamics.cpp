#include"sixdistaldynamics.h"
#include<stdio.h>
#include<math.h>
#include<array>
#include"robotconfig.h"
#include <aris.hpp>
using namespace sixDistalDynamicsInt;

using namespace CONFIG;
using namespace aris::plan;
using namespace aris::dynamic;


sixdistaldynamics::sixdistaldynamics()
        {
            A[0][0] = 0; A[0][1] = 1; A[0][2] = 0;
            A[1][0] = 0; A[1][1] = 0; A[1][2] = 1;
            A[2][0] = -CutoffFrequency * CutoffFrequency * CutoffFrequency;
            A[2][1] = -2 * CutoffFrequency * CutoffFrequency;
            A[2][2] = -2 * CutoffFrequency;
            B[0] = 0; B[1] = 0;
            B[2] = -A[2][0];
        }


void distalMatrix(const double* q, const double* dq, const double* ddq, const double* ts, double* distalVec)
{
    double t1, t2, t3, t4, t5, t6, t7, t8, t9, t10, t11, t12, t13, t14, t15, t16, t17,
        t18, t19, t20, t21, t22, t23, t24, t25, t26, t27, t28, t29, t30, t31, t32,
        t33, t34, t35, t36, t37, t38, t39, t40, t41, t42, t43, t44, t45, t46, t47,
        t48, t49, t50, t51, t52, t53, t54, t55, t56, t57, t58, t59, t60, t61, t62,
        t63, t64, t65, t66, t67, t68, t69, t70, t71, t72, t73, t74, t75, t76, t77,
        t78, t79, t80, t81, t82, t83, t84, t85, t86, t87, t88, t89, t90, t91, t92,
        t93, t94, t95, t96, t97, t98, t99, t100, t101, t102, t103, t104, t105,
        t106, t107, t108, t109, t110, t111, t112, t113, t114, t115, t116, t117,
        t118, t119, t120, t121, t122, t123, t124, t125, t126, t127, t128, t129,
        t130, t131, t132, t133, t134, t135, t136, t137, t138, t139, t140, t141,
        t142, t143, t144, t145, t146, t147, t148, t149, t150, t151, t152, t153,
        t154, t155, t156, t157, t158, t159, t160, t161, t162, t163, t164, t165,
        t166, t167, t168, t169, t170, t171, t172, t173, t174, t175, t176, t177,
        t178, t179, t180, t181, t182, t183, t184, t185, t186, t187, t188, t189,
        t190, t191, t192, t193, t194, t195, t196, t197, t198, t199, t200;
    //double[] qTemp=new double[common.RobotAxis];
      //  , dqTemp, ddqTemp;
    double q1, q2, q3, q4, q5, q6;
    double dq1, dq2, dq3, dq4, dq5, dq6;
    double ddq1, ddq2, ddq3, ddq4, ddq5, ddq6;
    double ts1, ts2, ts3, ts4, ts5, ts6;
    double g = 9.81;

    q1 = q[0];
    q2 = q[1];
    q3 = q[2];
    q4 = q[3];
    q5 = q[4];
    q6 = q[5];

    dq1 = dq[0]; dq2 = dq[1]; dq3 = dq[2]; dq4 = dq[3]; dq5 = dq[4]; dq6 = dq[5];
    ddq1 = ddq[0]; ddq2 = ddq[1]; ddq3 = ddq[2]; ddq4 = ddq[3]; ddq5 = ddq[4]; ddq6 = ddq[5];

    ts1 = ts[0]; ts2 = ts[1]; ts3 = ts[2]; ts4 = ts[3]; ts5 = ts[4]; ts6 = ts[5];

    double A0[6][GroupDim];
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < GroupDim; j++)
            A0[i][j] = 0;

    double DF = EndSensor[2];
    t2 = cos(q2);
    t3 = cos(q3);
    t4 = sin(q2);
    t5 = sin(q3);
    t7 = sin(q5);
    t8 = dq1*t2*t3;
    t9 = dq1*t4*t5;
    t10 = dq4-t8+t9;
    t11 = cos(q5);
    t12 = sin(q4);
    t13 = dq2+dq3;
    t14 = t12*t13;
    t15 = cos(q4);
    t16 = dq1*t2*t5;
    t17 = dq1*t3*t4;
    t18 = t16+t17;
    t19 = t15*t18;
    t20 = t14+t19;
    t39 = t7*t20;
    t40 = t10*t11;
    t6 = dq6+t39+t40;
    t22 = t13*t15;
    t23 = t12*t18;
    t24 = sin(q6);
    t25 = t7*t10;
    t26 = t11*t20;
    t27 = t25-t26;
    t28 = t24*t27;
    t29 = cos(q6);
    t30 = dq5+t22-t23;
    t31 = t29*t30;
    t21 = t28+t31;
    t32 = ddq1*t2;
    t47 = dq1*dq2*t4;
    t33 = t32-t47;
    t34 = ddq1*t4;
    t35 = dq1*dq2*t2;
    t36 = t34+t35;
    t37 = t24*t30;
    t41 = t27*t29;
    t38 = t37-t41;
    t42 = t22-t23;
    t43 = dq4*t42;
    t44 = t8-t9;
    t45 = dq3*t44;
    t46 = t3*t36;
    t48 = t5*t33;
    t49 = t45+t46+t48;
    t50 = t15*t49;
    t51 = ddq2+ddq3;
    t52 = t12*t51;
    t53 = t43+t50+t52;
    t54 = dq3*t18;
    t55 = t5*t36;
    t58 = t3*t33;
    t56 = ddq4+t54+t55-t58;
    t57 = dq1*dq1;
    t59 = t7*t56;
    t60 = t39+t40;
    t61 = dq5*t60;
    t96 = t11*t53;
    t62 = t59+t61-t96;
    t63 = t13*t13;
    t64 = a4*t44;
    t81 = d4*t18;
    t65 = t64-t81;
    t66 = dq2*dq2;
    t67 = a3*t66;
    t68 = g*t4;
    t69 = a2*t2*t57;
    t70 = t2*t2;
    t71 = a3*t57*t70;
    t72 = t67+t68+t69+t71;
    t73 = a3*ddq2;
    t74 = a2*t4*t57;
    t75 = a3*t2*t4*t57;
    t79 = g*t2;
    t76 = t73+t74+t75-t79;
    t77 = t15*t51;
    t97 = dq4*t20;
    t98 = t12*t49;
    t78 = ddq5+t77-t97-t98;
    t80 = a4*t63;
    t82 = t44*t65;
    t83 = t3*t72;
    t84 = d4*t51;
    t99 = t5*t76;
    t85 = t80+t82+t83+t84-t99;
    t86 = t54+t55-t58;
    t87 = a4*t86;
    t88 = d4*t49;
    t89 = a4*t13*t18;
    t90 = d4*t13*t44;
    t91 = a3*dq1*dq2*t4;
    t101 = a2*ddq1;
    t102 = a3*t33;
    t92 = t87+t88+t89+t90+t91-t101-t102;
    t93 = t21*t38;
    t94 = dq5*t27;
    t95 = t6*t6;
    t100 = DF*t62;
    t103 = DF*t30*t60;
    t133 = t12*t85;
    t134 = t15*t92;
    t104 = t100+t103-t133-t134;
    t105 = t18*t65;
    t106 = t5*t72;
    t107 = a4*t51;
    t108 = t3*t76;
    t120 = d4*t63;
    t109 = t105+t106+t107+t108-t120;
    t110 = DF*t78;
    t111 = t15*t85;
    t119 = t12*t92;
    t112 = t111-t119;
    t113 = t11*t112;
    t114 = dq6*t38;
    t115 = dq6*t21;
    t116 = t24*t78;
    t117 = t21*t21;
    t118 = t38*t38;
    t121 = t29*t62;
    t122 = t24*t62;
    t123 = t29*t78;
    t124 = t7*t53;
    t125 = t11*t56;
    t126 = ddq6+t93-t94+t124+t125;
    t127 = t6*t21;
    t128 = t7*t112;
    t129 = t30*t30;
    t130 = t27*t27;
    t131 = t11*t109;
    t138 = DF*t129;
    t139 = DF*t130;
    t132 = t128+t131-t138-t139;
    t140 = t7*t109;
    t141 = DF*t27*t60;
    t135 = t110+t113-t140-t141;
    t136 = t24*t135;
    t137 = t6*t38;
    t142 = -t24*t104-t29*t135;
    A0[0][6] = -t95-t117;
    A0[0][7] = -ddq6+t93+t94-t7*t53-t11*t56;
    A0[0][8] = t114-t6*t38-t24*t62-t29*t78;
    A0[0][9] = t142;
    A0[0][10] = 1.0;
    A0[1][6] = t126;
    A0[1][7] = -t95-t118;
    A0[1][8] = t115+t116-t6*t21-t29*t62;
    A0[1][9] = t136-t29*t104;
    A0[1][11] = 1.0;
    A0[2][6] = -t114+t122+t123-t6*t38;
    A0[2][7] = -t115-t116+t121-t6*t21;
    A0[2][8] = -t117-t118;
    A0[2][9] = t132;
    A0[2][12] = 1.0;
    A0[3][0] = -t115-t116+t121;
    A0[3][1] = t114-t122-t123+t137;
    A0[3][2] = t126;
    A0[3][3] = t127;
    A0[3][4] = -t95+t117;
    A0[3][5] = -t127;
    A0[3][7] = t132;
    A0[3][8] = -t136+t29*(t100+t103-t133-t134);
    A0[3][13] = 1.0;
    A0[4][0] = -t137;
    A0[4][1] = -t115-t116+t121-t127;
    A0[4][2] = t95-t118;
    A0[4][3] = t114-t122-t123;
    A0[4][4] = ddq6-t93-t94+t124+t125;
    A0[4][5] = t137;
    A0[4][6] = -t128-t131+t138+t139;
    A0[4][8] = t142;
    A0[4][14] = 1.0;
    A0[5][0] = -t93;
    A0[5][1] = -t117+t118;
    A0[5][2] = -t115-t116+t121+t127;
    A0[5][3] = t93;
    A0[5][4] = t114-t122-t123-t137;
    A0[5][5] = ddq6-t94+t124+t125;
    A0[5][6] = -t29*t104+t24*(t110+t113-t140-t141);
    A0[5][7] = t24*(t100+t103-t133-t134)+t29*(t110+t113-t140-t141);
    A0[5][15] = 1.0;

    for (int i = 0; i < 6; i++)
        for (int j = 0; j < GroupDim; j++)
           distalVec[GroupDim * i + j] = A0[i][j];

}

//inline int id(int m, int n, int cols) { return m * cols + n;}


void sixdistaldynamics::RLS(const double *positionL, const double *sensorL, double *estParas,double *StatisError)
{
    //positionList[id(2, 2, 6)];
    double stateMot0[6][3] = { 0 };
    double stateMot1[6][3] = { 0 };
    double stateTor0[6][3] = { 0 };
    double stateTor1[6][3] = { 0 };



    double q[6];
    double dq[6];
    double ddq[6];
    double ts[6];
    //std::array<double, 6> estParas;

    double intDT = DT;
    int length = 6;
    std::vector<double> regressorMatrix_vec(6 * SampleNum*GroupDim);
    auto regressorVector = regressorMatrix_vec.data();

    std::vector<double> regressorForces_vec(6 * SampleNum);
    auto regressorForces = regressorForces_vec.data();

    double posCur[6];
    double torCur[6];
    for (int j = 0; j < 6; j++)
    {
        stateMot0[j][0] = positionL[j];
        stateTor0[j][0] = sensorL[j];
    }

    for (int i = 0; i < SampleNum; i++)
    {

        for (int j = 0; j < 6; j++)
        {

            posCur[j] = positionL[6*i+j];
            torCur[j] = sensorL[6*i+j];

            stateMot1[j][0] = stateMot0[j][0] + intDT * (A[0][0] * stateMot0[j][0] + A[0][1] * stateMot0[j][1] + A[0][2] * stateMot0[j][2] + B[0] * posCur[j]);
            stateMot1[j][1] = stateMot0[j][1] + intDT * (A[1][0] * stateMot0[j][0] + A[1][1] * stateMot0[j][1] + A[1][2] * stateMot0[j][2] + B[1] * posCur[j]);
            stateMot1[j][2] = stateMot0[j][2] + intDT * (A[2][0] * stateMot0[j][0] + A[2][1] * stateMot0[j][1] + A[2][2] * stateMot0[j][2] + B[2] * posCur[j]);

            stateTor1[j][0] = stateTor0[j][0] + intDT * (A[0][0] * stateTor0[j][0] + A[0][1] * stateTor0[j][1] + A[0][2] * stateTor0[j][2] + B[0] * torCur[j]);
            stateTor1[j][1] = stateTor0[j][1] + intDT * (A[1][0] * stateTor0[j][0] + A[1][1] * stateTor0[j][1] + A[1][2] * stateTor0[j][2] + B[1] * torCur[j]);
            stateTor1[j][2] = stateTor0[j][2] + intDT * (A[2][0] * stateTor0[j][0] + A[2][1] * stateTor0[j][1] + A[2][2] * stateTor0[j][2] + B[2] * torCur[j]);
        }

        for (int j = 0; j < 6; j++)
        {
            q[j] = stateMot1[j][0];
            dq[j] = stateMot1[j][1];
            ddq[j] = stateMot1[j][2];
            ts[j] = stateTor1[j][0];
        }

        for (int k = 0; k < 6; k++)
        {
            q[k] = q[k] * DirectionFlag[k] + JointOffset[k] + ZeroOffset[k];
            dq[k] = dq[k] * DirectionFlag[k];
            ddq[k] = ddq[k] * DirectionFlag[k];

        }
        double distalVec[6 * GroupDim];
        distalMatrix(q, dq, ddq, ts, distalVec);
        double Y[6][GroupDim];
        for (int m = 0; m < 6; m++)
            for (int n = 0; n < GroupDim; n++)
                Y[m][n]=distalVec[GroupDim * m + n];

        for (int m = 0; m < 6; m++)
        {
            for (int n = 0; n < GroupDim; n++)
            {
                regressorVector[(i * 6 +m)*GroupDim +n] = Y[m][n];

            }
            regressorForces[i * 6 + m] = ts[m];

        }

        for (int j = 0; j < 6; j++)
        {

            stateMot0[j][0] = stateMot1[j][0];
            stateMot0[j][1] = stateMot1[j][1];
            stateMot0[j][2] = stateMot1[j][2];

            stateTor0[j][0] = stateTor1[j][0];
            stateTor0[j][1] = stateTor1[j][1];
            stateTor0[j][2] = stateTor1[j][2];
        }
        //std::cout<<i<<std::endl;
    }

    // 求解 A的广义逆pinv 和 x
    std::vector<double> pinv_vec(6 * SampleNum*GroupDim);
    auto pinv = pinv_vec.data();


    // 所需的中间变量，请对U的对角线元素做处理
    std::vector<double> U_vec(6 * SampleNum*GroupDim);
    auto U = U_vec.data();
    double tau[6 * SampleNum];
    aris::Size p[6 * SampleNum];
    aris::Size rank;

    // 根据 A 求出中间变量，相当于做 QR 分解 //
    // 请对 U 的对角线元素做处理
    s_householder_utp(6*SampleNum, GroupDim, regressorVector, U, tau, p, rank, 1e-10);

    // 根据QR分解的结果求广义逆，相当于Matlab中的 pinv(A) //
    double tau2[6*SampleNum];
    s_householder_utp2pinv( 6 * SampleNum,GroupDim, rank, U, tau, p, pinv, tau2, 1e-10);
    // 根据QR分解的结果求广义逆，相当于Matlab中的 pinv(A)*b //
    s_mm(GroupDim, 1, 6 * SampleNum, pinv, regressorForces, estParas);


    //for(int i=0;i<GroupDim;i++)
       // std::cout<<estParas[i]<<std::endl;
    //Calculate Model Error
    std::vector<double> Error_vec(6 * SampleNum);
    auto Error = Error_vec.data();

    s_mm(6 * SampleNum, 1, GroupDim, regressorVector,estParas, Error);
    for(int i=0;i<6*SampleNum;i++)
        Error[i]= Error[i]-regressorVector[i];

    double SumError[6]={0};
    for(int i=0;i<SampleNum;i++)
        for(int j=0;j<6;j++)
            SumError[j]= SumError[j]+Error[i*6+j]*Error[i*6+j];

    for(int j=0;j<6;j++)
        StatisError[j]= sqrt(SumError[j])/SampleNum;

    for(int i=0;i<6;i++)
        std::cout<<StatisError[i]<<std::endl;



    }



void sixdistaldynamics::sixDistalCollision(const double * q, const double *dq,const double *ddq,const double *ts, const double *estParas, double * estFT)
        {
            double estTor[6] = { 0, 0, 0, 0, 0, 0};

            double distalVec[6 * GroupDim];
            distalMatrix(q, dq, ddq, ts, distalVec);
            double Y[6][GroupDim];
            for (int m = 0; m < 6; m++)
                for (int n = 0; n < GroupDim; n++)
                    Y[m][n]=distalVec[GroupDim * m + n];


            for (int i = 0; i < 6; i++)
                for (int j = 0; j < GroupDim; j++)
                    estTor[i] = estTor[i] + Y[i][j] * estParas[j];
            for (int i = 0; i < 6; i++)
                estFT[i] = estTor[i] ;


        }




