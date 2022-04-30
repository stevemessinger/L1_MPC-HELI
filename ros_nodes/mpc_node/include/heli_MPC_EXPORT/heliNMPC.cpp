#include <acado_toolkit.hpp>
#include <acado_code_generation.hpp>
#include <acado_gnuplot.hpp>
#include <iostream>


int main(){
    USING_NAMESPACE_ACADO

    //Define Vehicle Dynamics
    DifferentialState x, y, z, qw, qx, qy, qz, xd, yd, zd, p, q, r;
    Control col, roll, pitch, yaw;


    double taup = 0.036889;
    double tauq = 0.06539;
    double taur = 0.083315;
    double Kp = 295.9283/taup * M_PI/180;
    double Kq = -299.1935/tauq * M_PI/180;
    double Kr = -299.1935/taur * M_PI/180;
    double mass = 0.362;
    double Kcol = 5*mass*9.81;

    auto Cbn13 = 2*(qx*qz+qw*qy);
    auto Cbn23 = 2*(qy*qz-qw*qx);
    auto Cbn33 = (qw*qw - qx*qx - qy*qy + qz*qz);

    DifferentialEquation f;
    f << dot(x) == xd;
    f << dot(y) == yd;
    f << dot(z) == zd;
    f << dot(qw) == 0.5 * (-qx*p - qy*q -qz*r);
    f << dot(qx) == 0.5 * (qw*p + qy*r - qz*q);
    f << dot(qy) == 0.5 * (qw*q - qx*r + qz*p);
    f << dot(qz) == 0.5 * (qw*r + qx*q - qy*p);
    f << dot(xd) == 1/mass * Cbn13 * Kcol * col;
    f << dot(yd) == 1/mass * Cbn23 * Kcol * col;
    f << dot(zd) == (1/mass * Cbn33 * Kcol * col + 9.81);
    f << dot(p) == (-1/taup * p + Kp*roll);
    f << dot(q) == (-1/tauq * q + Kq*pitch);
    f << dot(r) == (-1/taur * r + Kr*yaw);

    //Define LSQ
    Function h;
    h << x;
    h << y;
    h << z;
    h << qw;
    h << qx;
    h << qy;
    h << qz;
    h << xd;
    h << yd;
    h << zd;
    h << p;
    h << q;
    h << r;

    DMatrix Q(13,13);
    Q.setIdentity();
    Q(0,0) = 10.0;
    Q(1,1) = 10.0;
    Q(2,2) = 10.0;
    Q(3,3) = 5.0;
    Q(4,4) = 5.0;
    Q(5,5) = 5.0;
    Q(6,6) = 5.0;
    Q(7,7) = 0.05;
    Q(8,8) = 0.05;
    Q(9,9) = 0.05;
    Q(10,10) = 1.0;
    Q(11,11) = 1.0;
    Q(12,12) = 1.0;

    //Define the OCP
    const int startTime = 0;
    const int predictionHorizon = 1;
    const int freq = 10;

    OCP ocp(startTime, predictionHorizon, predictionHorizon*freq);
    ocp.minimizeLSQ(Q, h);
	ocp.minimizeLSQEndTerm(Q, h);

    ocp.subjectTo(f); // constrain to dynamics

    double colMax = 1.0;
    double rollMax = 1.0;
    double pitchMax = 1.0;
    double yawMax = 1.0;

    ocp.subjectTo(-colMax <= col <= colMax);
    ocp.subjectTo(-rollMax <= roll <= rollMax);
    ocp.subjectTo(-pitchMax <= pitch <= pitchMax);
    ocp.subjectTo(-yawMax <= yaw <= yawMax);

	// Export the code:
	OCPexport mpc( ocp );

	mpc.set( HESSIAN_APPROXIMATION,       GAUSS_NEWTON    );
	mpc.set( DISCRETIZATION_TYPE,         MULTIPLE_SHOOTING );
	mpc.set( INTEGRATOR_TYPE,             INT_RK4         );
	mpc.set( NUM_INTEGRATOR_STEPS,        30              );

	mpc.set( QP_SOLVER,                   QP_QPOASES      );
// 	mpc.set( HOTSTART_QP,                 YES             );
// 	mpc.set( LEVENBERG_MARQUARDT,         1.0e-4          );
	mpc.set( GENERATE_TEST_FILE,          YES             );
	mpc.set( GENERATE_MAKE_FILE,          YES             );
	mpc.set( GENERATE_MATLAB_INTERFACE,   YES             );
	mpc.set( GENERATE_SIMULINK_INTERFACE, YES             );

// 	mpc.set( USE_SINGLE_PRECISION,        YES             );

	if (mpc.exportCode( "heli_MPC_EXPORT" ) != SUCCESSFUL_RETURN)
		exit( EXIT_FAILURE );

	mpc.printDimensionsQP( );

    return 0;
}
















