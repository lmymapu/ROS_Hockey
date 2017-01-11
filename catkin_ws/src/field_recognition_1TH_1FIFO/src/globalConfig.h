#ifndef GLOBALCONFIG_H
#define GLOBALCONFIG_H
#define SIMULATION_MODE
//#define TEST_MODE

//params to objects in map
const double OBJ_POST_RADIUS = 0.015;
const double OBJ_PUCK_UP_RADIUS = 0.045;
const double OBJ_PUCK_LOW_RADIUS = 0.128;

//params to map config
const int MAP_NUMOF_POSTS = 14;
const int MAP_NUMOF_PUCKS = 3;

//params to motor control
const double MOTOR_ANGLE_CTRL_PREC = 0.05;
const double MOTOR_ANGLE_LOCK_PREC = 0.1;
const double MOTOR_POSE_CTRL_PREC = 0.05;

//params to laser detection
const double LASER_LEAST_INTENSITY = 0.1;
const double LASER_DETECT_THRES = 0.1;
const unsigned int LASER_MAX_OBJS = 25;
const double LASER_MAX_DETECT_RANGE = 1.0;
#ifdef SIMULATION_MODE
const double LASER_ANGLE_OFFSET=0;
#else
const double LASER_ANGLE_OFFSET=M_PI;
#endif

//params to camera detection
const double CAM_MID_REGION = 0.2;

//params to field recognition AI
const double RECAI_MINDIST_OBJ = 0.5;
const double RECAI_NORMAL_LINVEL = 0.2;
const double RECAI_NORMAL_ANGVEL = 0.6;

#endif // GLOBALCONFIG_H
