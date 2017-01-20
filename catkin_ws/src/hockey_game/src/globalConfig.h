#ifndef GLOBALCONFIG_H
#define GLOBALCONFIG_H
//#define DEBUG_ON
#define DEBUG_STATE_TRANS
#define DEBUG_MOTOR
//#define EASY_GAME_CONTROL
#define COMPLEX_GAME_CONTROL
//#define SIMULATION_MODE
#define TEST_MODE
//#define SINGLE_FIFO_TH_DATAREQ
#define MULTI_TH_DATAREQ
//#define MULTI_FIFO_DATAREQ
//#define MULTI_FIFO_TH_DATAREQ
//params to threading management
const int NUM_OF_DATAREQ_THREAD = 3;
const int NUM_OF_THREAD_MULTIFIFO = 3;

//params to objects in map
const double OBJ_POST_RADIUS = 0.03;
const double OBJ_PUCK_UP_RADIUS = 0.045;
const double OBJ_PUCK_LOW_RADIUS = 0.064;
const double OBJ_ROBOT_DIAMETER = 0.35;
const double OBJ_ROBOT_SAFE_RADIUS = 0.20;

//params to map config
const int MAP_NUMOF_POSTS = 14;
const int MAP_NUMOF_PUCKS = 3;
const double MAP_BORDER_X = 0.2;
const double MAP_BORDER_Y = 0.2;

//params to motor control
const double MOTOR_ANGLE_CTRL_PREC = 0.05;
const double MOTOR_ANGLE_LOCK_PREC = 0.1;
const double MOTOR_POSE_CTRL_PREC = 0.05;
const double MOTOR_CATCH_DIST = 0.02;
const double MOTOR_DANGER_DIST = 0.5;
const double MOTOR_SAFE_DIST = 1.2;

//params to laser detection
const int LASER_FIFO_LENGTH = 1;
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
const int CAM_FIFO_LENGTH = 1;
const double CAM_MID_REGION = 0.2;
const double CAM_FOV = 1.57;

//params to 3D camera detection
const int CAM3D_IMG_FIFO_LENGTH = 2;
const int CAM3D_PTCLOUD_FIFO_LENGTH = 2;
const double CAM3D_OBJ_DETECT_PREC = 0.1;
const float CAM3D_FLOOR_THRESHOLD = 0.51;

//params to field recognition AI
const double RECAI_MINDIST_OBJ = 0.5;
const double RECAI_NORMAL_LINVEL = 0.4;
const double RECAI_NORMAL_ANGVEL = 0.7;

//params to game AI
const double GAMEAI_MINDIST_OBJ = 0.4;
const double GAMEAI_NORMAL_LINVEL = 0.4;
const double GAMEAI_NORMAL_ANGVEL = 0.7;

#endif // GLOBALCONFIG_H
