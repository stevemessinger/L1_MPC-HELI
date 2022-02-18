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

    if (nrhs != 0){ 
      mexErrMsgTxt("This problem expects 0 right hand side argument(s) since you have defined 0 MexInput(s)");
    } 
 
    TIME autotime;
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
    DMatrix acadodata_M2;
    acadodata_M2.read( "simpleHeliMPC_LIVE_data_acadodata_M2.txt" );
    DVector acadodata_v3(13);
    acadodata_v3(0) = 0;
    acadodata_v3(1) = 0;
    acadodata_v3(2) = 0;
    acadodata_v3(3) = 1;
    acadodata_v3(4) = 0;
    acadodata_v3(5) = 0;
    acadodata_v3(6) = 0;
    acadodata_v3(7) = 0;
    acadodata_v3(8) = 0;
    acadodata_v3(9) = 0;
    acadodata_v3(10) = 0;
    acadodata_v3(11) = 0;
    acadodata_v3(12) = 0;
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
    acadodata_f1 << dot(p) == ((-2.00000000000000000000e+01)*p+2.51327412287183449280e+02*roll);
    acadodata_f1 << dot(q) == ((-2.00000000000000000000e+01)*q+2.51327412287183449280e+02*pitch);
    acadodata_f1 << dot(r) == ((-2.00000000000000000000e+01)*r+2.51327412287183449280e+02*yaw);

    OCP ocp1(0, 5, 50);
    ocp1.minimizeLSQ(acadodata_M1, acadodata_f2, acadodata_v2);
    ocp1.subjectTo(acadodata_f1);
    ocp1.subjectTo((-1.00000000000000000000e+00) <= col <= 1.00000000000000000000e+00);
    ocp1.subjectTo((-1.00000000000000000000e+00) <= roll <= 1.00000000000000000000e+00);
    ocp1.subjectTo((-1.00000000000000000000e+00) <= pitch <= 1.00000000000000000000e+00);
    ocp1.subjectTo((-1.00000000000000000000e+00) <= yaw <= 1.00000000000000000000e+00);


    RealTimeAlgorithm algo1(ocp1, 0.1);
    algo1.set( INTEGRATOR_TYPE, INT_RK45 );
    algo1.set( INTEGRATOR_TOLERANCE, 1.000000E-06 );
    algo1.set( ABSOLUTE_TOLERANCE, 1.000000E-04 );
    algo1.set( MAX_NUM_ITERATIONS, 2 );
    algo1.set( HESSIAN_APPROXIMATION, GAUSS_NEWTON );

    StaticReferenceTrajectory referencetrajectory(acadodata_M2);
    Controller controller1( algo1,referencetrajectory );

    OutputFcn acadodata_f3;

    DynamicSystem dynamicsystem2( acadodata_f1,acadodata_f3 );
    Process process3( dynamicsystem2,INT_RK45 );

    SimulationEnvironment algo2(0, 20, process3, controller1);
     algo2.init(acadodata_v3);
    returnValue returnvalue = algo2.run();


    VariablesGrid out_processout; 
    VariablesGrid out_feedbackcontrol; 
    VariablesGrid out_feedbackparameter; 
    VariablesGrid out_states; 
    VariablesGrid out_algstates; 
    algo2.getSampledProcessOutput(out_processout);
    algo2.getProcessDifferentialStates(out_states);
    algo2.getFeedbackControl(out_feedbackcontrol);
    const char* outputFieldNames[] = {"STATES_SAMPLED", "CONTROLS", "PARAMETERS", "STATES", "ALGEBRAICSTATES", "CONVERGENCE_ACHIEVED"}; 
    plhs[0] = mxCreateStructMatrix( 1,1,6,outputFieldNames ); 
    mxArray *OutSS = NULL;
    double  *outSS = NULL;
    OutSS = mxCreateDoubleMatrix( out_processout.getNumPoints(),1+out_processout.getNumValues(),mxREAL ); 
    outSS = mxGetPr( OutSS );
    for( int i=0; i<out_processout.getNumPoints(); ++i ){ 
      outSS[0*out_processout.getNumPoints() + i] = out_processout.getTime(i); 
      for( int j=0; j<out_processout.getNumValues(); ++j ){ 
        outSS[(1+j)*out_processout.getNumPoints() + i] = out_processout(i, j); 
       } 
    } 

    mxSetField( plhs[0],0,"STATES_SAMPLED",OutSS );
    mxArray *OutS = NULL;
    double  *outS = NULL;
    OutS = mxCreateDoubleMatrix( out_states.getNumPoints(),1+out_states.getNumValues(),mxREAL ); 
    outS = mxGetPr( OutS );
    for( int i=0; i<out_states.getNumPoints(); ++i ){ 
      outS[0*out_states.getNumPoints() + i] = out_states.getTime(i); 
      for( int j=0; j<out_states.getNumValues(); ++j ){ 
        outS[(1+j)*out_states.getNumPoints() + i] = out_states(i, j); 
       } 
    } 

    mxSetField( plhs[0],0,"STATES",OutS );
    mxArray *OutC = NULL;
    double  *outC = NULL;
    OutC = mxCreateDoubleMatrix( out_feedbackcontrol.getNumPoints(),1+out_feedbackcontrol.getNumValues(),mxREAL ); 
    outC = mxGetPr( OutC );
    for( int i=0; i<out_feedbackcontrol.getNumPoints(); ++i ){ 
      outC[0*out_feedbackcontrol.getNumPoints() + i] = out_feedbackcontrol.getTime(i); 
      for( int j=0; j<out_feedbackcontrol.getNumValues(); ++j ){ 
        outC[(1+j)*out_feedbackcontrol.getNumPoints() + i] = out_feedbackcontrol(i, j); 
       } 
    } 

    mxSetField( plhs[0],0,"CONTROLS",OutC );
    mxArray *OutP = NULL;
    double  *outP = NULL;
    OutP = mxCreateDoubleMatrix( out_feedbackparameter.getNumPoints(),1+out_feedbackparameter.getNumValues(),mxREAL ); 
    outP = mxGetPr( OutP );
    for( int i=0; i<out_feedbackparameter.getNumPoints(); ++i ){ 
      outP[0*out_feedbackparameter.getNumPoints() + i] = out_feedbackparameter.getTime(i); 
      for( int j=0; j<out_feedbackparameter.getNumValues(); ++j ){ 
        outP[(1+j)*out_feedbackparameter.getNumPoints() + i] = out_feedbackparameter(i, j); 
       } 
    } 

    mxSetField( plhs[0],0,"PARAMETERS",OutP );
    mxArray *OutZ = NULL;
    double  *outZ = NULL;
    OutZ = mxCreateDoubleMatrix( out_algstates.getNumPoints(),1+out_algstates.getNumValues(),mxREAL ); 
    outZ = mxGetPr( OutZ );
    for( int i=0; i<out_algstates.getNumPoints(); ++i ){ 
      outZ[0*out_algstates.getNumPoints() + i] = out_algstates.getTime(i); 
      for( int j=0; j<out_algstates.getNumValues(); ++j ){ 
        outZ[(1+j)*out_algstates.getNumPoints() + i] = out_algstates(i, j); 
       } 
    } 

    mxSetField( plhs[0],0,"ALGEBRAICSTATES",OutZ );
    mxArray *OutConv = NULL;
    if ( returnvalue == SUCCESSFUL_RETURN ) { OutConv = mxCreateDoubleScalar( 1 ); }else{ OutConv = mxCreateDoubleScalar( 0 ); } 
    mxSetField( plhs[0],0,"CONVERGENCE_ACHIEVED",OutConv );


    clearAllStaticCounters( ); 
 
} 

