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
#include "heli_messages/Pose.h"
#include "heli_messages/Inputs.h"
#include "heli_messages/Trajectory.h"

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

    void poseCallback(const heli_messages::Pose::ConstPtr& poseMessage);

    void trajectoryCallback(const heli_messages::Trajectory::ConstPtr& trajectoryMessage);

    ros::NodeHandle nh;
    ros::Subscriber poseSubscriber;
    ros::Subscriber trajSubscriber;
    ros::Publisher inputPublisher;

    ACADOvariables acadoVariables;
    ACADOworkspace acadoWorkspace;

    heli_messages::Inputs inputs;

    bool verbose;
};
