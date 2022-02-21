/*
*    This file is part of ACADO Toolkit.
*
*    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
*    Copyright (C) 2008-2009 by Boris Houska and Hans Joachim Ferreau, K.U.Leuven.
*    Developed within the Optimization in Engineering Center (OPTEC) under
*    supervision of Moritz Diehl. All rights reserved.
*
*    ACADO Toolkit is free software; you can redistribute it and/or
*    modify it under the terms of the GNU Lesser General Public
*    License as published by the Free Software Foundation; either
*    version 3 of the License, or (at your option) any later version.
*
*    ACADO Toolkit is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
*    Lesser General Public License for more details.
*
*    You should have received a copy of the GNU Lesser General Public
*    License along with ACADO Toolkit; if not, write to the Free Software
*    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*
*/


/**
*    Author David Ariens, Rien Quirynen
*    Date 2009-2013
*    http://www.acadotoolkit.org/matlab 
*/

#include <acado_optimal_control.hpp>
#include <acado_toolkit.hpp>
#include <acado/utils/matlab_acado_utils.hpp>

USING_NAMESPACE_ACADO

#include <mex.h>


void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[] ) 
 { 
 
    MatlabConsoleStreamBuf mybuf;
    RedirectStream redirect(std::cout, mybuf);
    clearAllStaticCounters( ); 
 
    mexPrintf("\nACADO Toolkit for Matlab - Developed by David Ariens and Rien Quirynen, 2009-2013 \n"); 
    mexPrintf("Support available at http://www.acadotoolkit.org/matlab \n \n"); 

    if (nrhs != 3){ 
      mexErrMsgTxt("This problem expects 3 right hand side argument(s) since you have defined 3 MexInput(s)");
    } 
 
    TIME autotime;
    int mexinput0_count = 0;
    if (mxGetM(prhs[0]) == 1 && mxGetN(prhs[0]) >= 1) 
       mexinput0_count = mxGetN(prhs[0]);
    else if (mxGetM(prhs[0]) >= 1 && mxGetN(prhs[0]) == 1) 
       mexinput0_count = mxGetM(prhs[0]);
    else 
       mexErrMsgTxt("Input 0 must be a noncomplex double vector of dimension 1xY.");

    double *mexinput0_temp = NULL; 
    if( !mxIsDouble(prhs[0]) || mxIsComplex(prhs[0])) { 
      mexErrMsgTxt("Input 0 must be a noncomplex double vector of dimension 1xY.");
    } 
    mexinput0_temp = mxGetPr(prhs[0]); 
    DVector mexinput0(mexinput0_count);
    for( int i=0; i<mexinput0_count; ++i ){ 
        mexinput0(i) = mexinput0_temp[i];
    } 

    double *mexinput1_temp = NULL; 
    if( !mxIsDouble(prhs[1]) || mxIsComplex(prhs[1]) || !(mxGetM(prhs[1])==1 && mxGetN(prhs[1])==1) ) { 
      mexErrMsgTxt("Input 1 must be a noncomplex scalar double.");
    } 
    mexinput1_temp = mxGetPr(prhs[1]); 
    double mexinput1 = *mexinput1_temp; 

    double *mexinput2_temp = NULL; 
    if( !mxIsDouble(prhs[2]) || mxIsComplex(prhs[2]) ) { 
      mexErrMsgTxt("Input 2 must be a noncomplex double vector of dimension XxY.");
    } 
    mexinput2_temp = mxGetPr(prhs[2]); 
    DMatrix mexinput2(mxGetM(prhs[2]), mxGetN(prhs[2]));
    for( int i=0; i<mxGetN(prhs[2]); ++i ){ 
        for( int j=0; j<mxGetM(prhs[2]); ++j ){ 
           mexinput2(j,i) = mexinput2_temp[i*mxGetM(prhs[2]) + j];
        } 
    } 

    DifferentialState x;
    DifferentialState y;
    DifferentialState z;
    DifferentialState qw;
    DifferentialState qx;
    DifferentialState qy;
    DifferentialState qz;
    DifferentialState xd;
    DifferentialState yd;
    DifferentialState zd;
    DifferentialState p;
    DifferentialState q;
    DifferentialState r;
    Control col;
    Control roll;
    Control pitch;
    Control yaw;
    Function acadodata_f2;
    acadodata_f2 << x;
    acadodata_f2 << y;
    acadodata_f2 << z;
    acadodata_f2 << qw;
    acadodata_f2 << qx;
    acadodata_f2 << qy;
    acadodata_f2 << qz;
    acadodata_f2 << xd;
    acadodata_f2 << yd;
    acadodata_f2 << zd;
    acadodata_f2 << p;
    acadodata_f2 << q;
    acadodata_f2 << r;
    DMatrix acadodata_M1;
    acadodata_M1.read( "simpleHeliMPC_LIVE_data_acadodata_M1.txt" );
    DVector acadodata_v1(13);
    acadodata_v1(0) = 0;
    acadodata_v1(1) = 0;
    acadodata_v1(2) = 0;
    acadodata_v1(3) = 1;
    acadodata_v1(4) = 0;
    acadodata_v1(5) = 0;
    acadodata_v1(6) = 0;
    acadodata_v1(7) = 0;
    acadodata_v1(8) = 0;
    acadodata_v1(9) = 0;
    acadodata_v1(10) = 0;
    acadodata_v1(11) = 0;
    acadodata_v1(12) = 0;
    DVector acadodata_v2(13);
    acadodata_v2(0) = 0;
    acadodata_v2(1) = 0;
    acadodata_v2(2) = 0;
    acadodata_v2(3) = 1;
    acadodata_v2(4) = 0;
    acadodata_v2(5) = 0;
    acadodata_v2(6) = 0;
    acadodata_v2(7) = 0;
    acadodata_v2(8) = 0;
    acadodata_v2(9) = 0;
    acadodata_v2(10) = 0;
    acadodata_v2(11) = 0;
    acadodata_v2(12) = 0;
    DifferentialEquation acadodata_f1;
    acadodata_f1 << dot(x) == xd;
    acadodata_f1 << dot(y) == yd;
    acadodata_f1 << dot(z) == zd;
    acadodata_f1 << dot(qw) == ((-qx)*p-q*qy-qz*r)*5.00000000000000000000e-01;
    acadodata_f1 << dot(qx) == (p*qw-q*qz+qy*r)*5.00000000000000000000e-01;
    acadodata_f1 << dot(qy) == (p*qz+q*qw-qx*r)*5.00000000000000000000e-01;
    acadodata_f1 << dot(qz) == (-p*qy+q*qx+qw*r)*5.00000000000000000000e-01;
    acadodata_f1 << dot(xd) == ((-2.00000000000000000000e+01)+1.00000000000000000000e+02*col)*(qw*qy+qx*qz)*2.00000000000000000000e+00;
    acadodata_f1 << dot(yd) == ((-2.00000000000000000000e+01)+1.00000000000000000000e+02*col)*(-qw*qx+qy*qz)*2.00000000000000000000e+00;
    acadodata_f1 << dot(zd) == (((-2.00000000000000000000e+01)+1.00000000000000000000e+02*col)*(pow(qw,2.00000000000000000000e+00)-pow(qx,2.00000000000000000000e+00)-pow(qy,2.00000000000000000000e+00)+pow(qz,2.00000000000000000000e+00))+9.81000000000000049738e+00);
    acadodata_f1 << dot(p) == ((-2.00000000000000000000e+01)*p+3.49065850398865904936e+02*roll);
    acadodata_f1 << dot(q) == ((-2.00000000000000000000e+01)*q+3.49065850398865904936e+02*pitch);
    acadodata_f1 << dot(r) == ((-2.00000000000000000000e+01)*r+3.49065850398865904936e+02*yaw);

    OCP ocp1(0, 1, 10);
    ocp1.minimizeLSQ(acadodata_M1, acadodata_f2, acadodata_v2);
    ocp1.subjectTo(acadodata_f1);
    ocp1.subjectTo((-1.00000000000000000000e+00) <= col <= 1.00000000000000000000e+00);
    ocp1.subjectTo((-1.00000000000000000000e+00) <= roll <= 1.00000000000000000000e+00);
    ocp1.subjectTo((-1.00000000000000000000e+00) <= pitch <= 1.00000000000000000000e+00);
    ocp1.subjectTo((-1.00000000000000000000e+00) <= yaw <= 1.00000000000000000000e+00);


    RealTimeAlgorithm algo1(ocp1, 0.02);
    algo1.set( INTEGRATOR_TYPE, INT_RK45 );
    algo1.set( INTEGRATOR_TOLERANCE, 1.000000E-06 );
    algo1.set( ABSOLUTE_TOLERANCE, 1.000000E-04 );
    algo1.set( MAX_NUM_ITERATIONS, 2 );
    algo1.set( HESSIAN_APPROXIMATION, GAUSS_NEWTON );

    StaticReferenceTrajectory referencetrajectory(mexinput2);
    Controller controller1( algo1,referencetrajectory );
    controller1.init(mexinput1, mexinput0);
    controller1.step(mexinput1, mexinput0);

    const char* outputFieldNames[] = {"U", "P"}; 
    plhs[0] = mxCreateStructMatrix( 1,1,2,outputFieldNames ); 
    mxArray *OutU = NULL;
    double  *outU = NULL;
    OutU = mxCreateDoubleMatrix( 1,controller1.getNU(),mxREAL ); 
    outU = mxGetPr( OutU );
    DVector vec_outU; 
    controller1.getU(vec_outU); 
    for( int i=0; i<vec_outU.getDim(); ++i ){ 
        outU[i] = vec_outU(i); 
    } 

    mxArray *OutP = NULL;
    double  *outP = NULL;
    OutP = mxCreateDoubleMatrix( 1,controller1.getNP(),mxREAL ); 
    outP = mxGetPr( OutP );
    DVector vec_outP; 
    controller1.getP(vec_outP); 
    for( int i=0; i<vec_outP.getDim(); ++i ){ 
        outP[i] = vec_outP(i); 
    } 

    mxSetField( plhs[0],0,"U",OutU );
    mxSetField( plhs[0],0,"P",OutP );


    clearAllStaticCounters( ); 
 
} 

