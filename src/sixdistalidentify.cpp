#include"sixdistalidentify.h"
#include<stdio.h>
#include<math.h>
#include"robotconfig.h"
#include<array>
using namespace sixDistalIdentifyInt;
using namespace CONFIG;

sixdistalidentify::sixdistalidentify()
        {

            //sixDistalMatrix = new sixDistalDynamics();
           // matrixList=new List<double[,]> ();
            //forceList = new List<double[]>();
            A[0][0] = 0; A[0][1] = 1; A[0][2] = 0;
            A[1][0] = 0; A[1][1] = 0; A[1][2] = 1;
            A[2][0] = -cutoffFreq * cutoffFreq * cutoffFreq;
            A[2][1] = -2 * cutoffFreq * cutoffFreq;
            A[2][2] = -2 * cutoffFreq;
            B[0] = 0; B[1] = 0;
            B[2] = -A[2][0];
        }
std::array<double, 6> sixdistalidentify::RLS(std::array<double, 6> &positionList, std::array<double, 6> &sensorList)
        {
			double stateMot0[6][3];
			double stateMot1[6][3];
			double stateTor0[6][3];
			double stateTor1[6][3];
            double intDT = DT;
            int length = 6;
            regressorMatrix = new Matrix(common.RobotAxis * length, common.GroupDim);
            regressorForces = new Matrix(common.RobotAxis * length, 1);
            double posCur[6];
            double torCur[6];
            for (int j = 0; j < common.RobotAxis; j++)
            {
                stateMot0[j][0] = positionList[j];
                stateTor0[j][0] = sensorList[j];
            }
    
            for (int i = 0; i < length; i++)
            {

                for (int j = 0; j < 6; j++)
                { 
                   
                    posCur[j] = positionList[i,j];
                    torCur[j] = sensorList[i,j];

                    stateMot1[j][0] = stateMot0[j, 0] + intDT * (A[0][0] * stateMot0[j][0] + A[0][1] * stateMot0[j][1] + A[0][2] * stateMot0[j][2] + B[0] * posCur[j]);
                    stateMot1[j][1] = stateMot0[j, 1] + intDT * (A[1][0] * stateMot0[j][0] + A[1][1] * stateMot0[j][1] + A[1][2] * stateMot0[j][2] + B[1] * posCur[j]);
                    stateMot1[j][2] = stateMot0[j, 2] + intDT * (A[2][0] * stateMot0[j][0] + A[2][1] * stateMot0[j][1] + A[2][2] * stateMot0[j][2] + B[2] * posCur[j]);

                    stateTor1[j][0] = stateTor0[j, 0] + intDT * (A[0][0] * stateTor0[j][0] + A[0][1] * stateTor0[j][1] + A[0][2] * stateTor0[j][2] + B[0] * torCur[j]);
                    stateTor1[j][1] = stateTor0[j, 1] + intDT * (A[1][0] * stateTor0[j][0] + A[1][1] * stateTor0[j][1] + A[1][2] * stateTor0[j][2] + B[1] * torCur[j]);
                    stateTor1[j][2] = stateTor0[j, 2] + intDT * (A[2][0] * stateTor0[j][0] + A[2][1] * stateTor0[j][1] + A[2][2] * stateTor0[j][2] + B[2] * torCur[j]);
                }

                for (int j = 0; j < common.RobotAxis; j++)
                {
                    q[j] = stateMot1[j][0];
                    dq[j] = stateMot1[j][1];
                    ddq[j] = stateMot1[j][2];
                    ts[j] = stateTor1[j][0];
                }

                double Y[6][GroupDim];
                Y = distalMatrix.distalMatrix(q, dq, ddq, ts);
                for (int m = 0; m < common.RobotAxis/2; m++)
                {
                    for (int n = 0; n < common.GroupDim; n++)
                    {
                        regressorMatrix[i * common.RobotAxis + m, n] = Y[m, n];

                    }
                    regressorForces[i * common.RobotAxis + m, 0] = ts[m];

                }
                //matrixList.Add(Y);
                //forceList.Add(ts);
                for (int j = 0; j < common.RobotAxis; j++)
                {

                    stateMot0[j][0] = stateMot1[j][0];
                    stateMot0[j][1] = stateMot1[j][1];
                    stateMot0[j][2] = stateMot1[j][2];

                    stateTor0[j][0] = stateTor1[j][0];
                    stateTor0[j][1] = stateTor1[j][1];
                    stateTor0[j][2] = stateTor1[j][2];
                }

            }
          
            Matrix regressorMatrixTrans = Matrix.Transpose(regressorMatrix);
            Matrix t = regressorMatrixTrans * regressorMatrix;
            Matrix invt = t.Invert();
            Matrix paras = invt * regressorMatrixTrans * regressorForces;
            Matrix error = regressorMatrix * paras - regressorForces;

            for (int i = 0; i < common.GroupDim; i++)
                estParas[i] = paras[i, 0];
            return estParas;
        }



       

