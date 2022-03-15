#include "heli_MPC_EXPORT/acado_common.h"
#include "heli_MPC_EXPORT/acado_auxiliary_functions.h"
#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <string>
#include <iomanip>
#include <cstring>
#include <cmath>
#include <cstdlib>
#include "ros/ros.h"
#include "ros/time.h"
#include "ros/console.h"
#include "mpc_node/Pose.h"
#include "mpc_node/Inputs.h"
#include "mpc_node/Trajectory.h"

/* Some convenient definitions. */
#define NX          ACADO_NX  /* Number of differential state variables.  */
#define NXA         ACADO_NXA /* Number of algebraic variables. */
#define NU          ACADO_NU  /* Number of control inputs. */
#define NOD         ACADO_NOD  /* Number of online data values. */

#define NY          ACADO_NY  /* Number of measurements/references on nodes 0..N - 1. */
#define NYN         ACADO_NYN /* Number of measurements/references on node N. */

#define N           ACADO_N   /* Number of intervals in the horizon. */

class mpcController{
public:

    mpcController();

    mpcController(bool);

    ~mpcController();

    bool init();

    bool loop();

private:

    void poseCallback(const mpc_node::Pose::ConstPtr& poseMessage);

    void trajectoryCallback(const mpc_node::Trajectory::ConstPtr& trajectoryMessage);

    ros::NodeHandle nh;
    ros::Subscriber poseSubscriber;
    ros::Subscriber trajSubscriber;
    ros::Publisher inputPublisher;

    ACADOvariables acadoVariables;
    ACADOworkspace acadoWorkspace;

    mpc_node::Inputs inputs;

    bool verbose;
};
