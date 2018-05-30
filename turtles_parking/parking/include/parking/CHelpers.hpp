// CHelpers.hpp

#ifndef __CHELPERS_HPP
#define __CHELPERS_HPP

// D E F I N E S ---------------------------------------------------------------------
#define PARKING_LOOP_RATE           10

#define IMU_CALLBACK_TRESHOLD       0.001
#define HALL_TICKS_PER_TURN         8.0
#define WHEEL_DIAMATER              0.084

#define PID_P                       70.0
#define PID_I                       5.0
#define PID_D                       80.0
#define PID_DT                      1.0 / PARKING_LOOP_RATE

#define LOCKED_TO_WALL_THRESHOLD    0.03  // In this area US-sensor dist must be to ock to wall in scouting
#define SECS_FOR_LOCK_TO_WALL       2.5     // Seconds to lock to wall in scouting
#define NEW_TARGET_SET_TRESH        0.06  // US-sensor tresh to detect new Targets
#define DIST_TO_DRIVE_AFTER_LOT     0.50  // Dist to drive after detecting a lot (before park-process)
#define PARKING_SPEED               220   // Motorspeed during the park process
#define SCOUTING_SPEED              200   // Motorspeed during scouting process
#define MAX_SCOUTING_STEERING       700   // Max-steering for pid-controller in scouting
#define SAFETY_DIST                 0.1   // Safety distance to wall during the park process
#define COLLISION_DIST              0.05  // Safety distance to everything in [m]
#define MIN_PARKING_LOT_LENGTH_T1   0.90  // Minimal pl-width for two circle parking in [m]
#define MIN_PARKING_LOT_DEPTH_T1    0.30  // Minimal pl-height for two circle parking in [m]
#define MIN_PARKING_LOT_LENGTH_T2   0.35  // Minimal pl-width for one circle parking in [m]
#define MIN_PARKING_LOT_HEIGHT_T2   0.50  // Minimal pl-height for one circle parking in [m]

#define CAR_LENGTH                  0.440
#define CAR_WIDTH                   0.265
#define CAR_HEIGHT                  0.340
#define DIST_REAR_AXLE_TO_FRONT     0.350
#define DIST_REAR_AXLE_TO_BACK      0.090
#define DIST_FRONT_AXLE_TO_FRONT    0.095
#define DIST_FRONT_AXLE_TO_BACK     0.350

#define MINIMAL_STEERING_RADIUS     0.930
#define STEER_FOR_MIN_RAD_RIGHT     700
#define STEER_FOR_MIN_RAD_LEFT      -700

#define WALL_FOLLOW_DISTANCE        0.45  // distance to the wall

#define EMERGENCY_FRONT_TRESHOLD    0.20
#define EMERGENCY_STOP_TICKS        1000    // 10 hz * 10

// M A K R O S -----------------------------------------------------------------------
#define degreesToRadians(deg) (deg * M_PI / 180.0)
#define radiansToDegrees(rad) (rad * 180.0 / M_PI)

// E N U M S -------------------------------------------------------------------------
enum PCB { PCB_OK,                        // Everything fine
           PCB_ERROR,                     // Error happend
           PCB_SCOUTING_FINISHED,         // Scouting finished
           PCB_PARKING_LOT_TOO_SMALL,     // Parking lot is too small for parking
           PCB_PARKING_FINISHED           // ParkProcessFinished
}; // ENUM PCB

enum PLType { PLT_LONG,                   // Reverse parking with two circles
              PLT_SMALL                   // Reverse parking with only on circle
}; // ENUM PLTYPE

// S T R U C T S ---------------------------------------------------------------------
struct SParkingLot {
    float fPLWidth;                       // Width of the parking-lot
    float fPLHeight;                      // Height of the parking-lot

    PLType Type;                          // Type of this parking-lot

    float fTheta;
}; // STRUCT SPARKINGLOT

#endif // __CHELPERS_HPP
