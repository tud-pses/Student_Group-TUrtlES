// CScouting.cpp

// I N C L U D E S -------------------------------------------------------------------
#include <math.h>

// O W N - I N C L U D E S -----------------------------------------------------------
#include "CScouting.hpp"

// C L A S S - C S C O U T I N G ----------------------------------------------------
CScouting::CScouting(CSensors &sensors, CMovement &movement, CScoutingCB callback):
    m_Sensors(sensors),
    m_Movement(movement),
    m_callback(callback) {

    std::unique_ptr<CPIDController> controller(new CPIDController(
          PID_P, PID_I, PID_D, PID_DT, MAX_SCOUTING_STEERING, -MAX_SCOUTING_STEERING));
    m_distController = std::move(controller);

    m_Status          = SS_FIND_WALL;
    m_TargetDistance  = WALL_FOLLOW_DISTANCE;
    m_FirstBox        = true;
    m_WallFoundCnt    = 0;
    m_DrivenDist      = 0.0;
    m_ParkingLotWidth = 0.0;
    m_LotDepth        = 0.0;

    m_Movement.setMotorSpeed(SCOUTING_SPEED);
} // CONSTRUCTOR

PCB CScouting::spin() {
    switch(m_Status) {
        case SS_FIND_WALL: {
            findWall();
            break;
        } // SS_FIND_WALL
        case SS_FOLLOW_WALL: {
            m_Sensors.startTrackingYaw();
            followWall();
            break;
        } // SS_FOLLOW_WALL
        case SS_PREPARE_FOR_PARKING: {
            followWall();
            prepareForParking();
            break;
        } // SS_PREPARE_FOR_PARKING
        case SS_FINISHED: {
            m_Sensors.stopTrackingYaw();
            SParkingLot parkingLot;
            parkingLot.fTheta    = calcTheta();
            parkingLot.fPLWidth  = m_ParkingLotWidth;
            parkingLot.fPLHeight = m_LotDepth;

            ROS_INFO("Parking length %f", m_ParkingLotWidth);
            m_callback(parkingLot);
            break;
        } // SS_FINISHED
    } // SWITCH

    return PCB_OK;
} // SPIN

void CScouting::findWall() {
    double distance = m_Sensors.getDistRight();
    double steering = m_distController->compute(0, (distance-m_TargetDistance)*100);
    m_Movement.setSteeringLevel(-steering);

    double delta = distance-m_TargetDistance;

    if (m_WallFoundCnt > PARKING_LOOP_RATE * SECS_FOR_LOCK_TO_WALL) {
        m_Status        = SS_FOLLOW_WALL;
        m_WallFoundCnt  = 0;

        ROS_INFO("  Scouting: Locked to wall");

        return;
    } // IF

    if (fabs(delta) <= LOCKED_TO_WALL_THRESHOLD)
        m_WallFoundCnt++;
    else
        m_WallFoundCnt = 0;
} // FINDWALL

void CScouting::followWall() {
    double distance = m_Sensors.getDistRight();
    double delta    = distance-m_TargetDistance;

    if (fabs(delta) >= NEW_TARGET_SET_TRESH) {
        if (m_Status != SS_PREPARE_FOR_PARKING)
          registerBox(delta);

        // Correct minimal deviation caused by pid-controller
        if (distance >= WALL_FOLLOW_DISTANCE-0.02)
            m_TargetDistance = WALL_FOLLOW_DISTANCE;
        else
            m_TargetDistance = std::min(distance, WALL_FOLLOW_DISTANCE);
        m_distController->reset();
    } // IF

    // PID only every 2 ticks to minimize false reading from us
    m_PIDCnt++;
    if (m_PIDCnt >= 2) {
        double steering = m_distController->compute(0, delta*100);
        m_Movement.setSteeringLevel(-steering);
        m_PIDCnt = 0;
    } // IF
} // FOLLOWWALL

void CScouting::registerBox(double delta) {
    if (delta < 0) {
        // Ignore first obstacle (we want to park between two obstacles, not behind the first)
        if (!m_FirstBox) {
            double length = m_Sensors.getDrivenDist() - m_LastBox;

            // If parkinglot length too small drive further
            if (length >= MIN_PARKING_LOT_LENGTH_T1 && m_TargetDistance >= MIN_PARKING_LOT_DEPTH_T1) {
                m_DrivenDist      = m_Sensors.getDrivenDist();
                m_Status          = SS_PREPARE_FOR_PARKING;
                m_LotDepth        = m_TargetDistance;
                m_ParkingLotWidth = length;
            } // IF

            ROS_INFO("  Scouting: Parking-lot length: %f depth: %f", length, m_LotDepth);
        } // IF
    } // IF
    else {
        m_LastBox  = m_Sensors.getDrivenDist();
        m_FirstBox = false;
    } // ELSE
} // REGISTERBOX

void CScouting::prepareForParking() {
    double delta = m_Sensors.getDrivenDist() - m_DrivenDist;

    // Drive further to minimize lot-length
    if (delta >= DIST_TO_DRIVE_AFTER_LOT) {
        m_Status = SS_FINISHED;
        m_Movement.setMotorSpeed(0);
        m_Movement.setSteeringLevel(0);
    } // IF
} // PREPAREFORPARKING

double CScouting::calcTheta() {
    float a = m_LotDepth - 0.185;        // -0.185 needed for compensate steering irregularities
    float b = DIST_REAR_AXLE_TO_FRONT;
    float R = MINIMAL_STEERING_RADIUS;

    float R_0 = (b*b) / (2*a);

    // Correct R_0 if necessary
    if (R_0 < R) {
        R_0 = R;
        b   = sqrt(2*a / R);
    } // IF

    float theta = acos(1 - (a / (R + R_0)));

    ROS_INFO("  Scouting: Calculated theta(in deg) for park-process: %f", radiansToDegrees(theta));

    return theta;
/*
    // Das hier ist aggressives einparken
    double R_e   = sqrt(pow(MINIMAL_STEERING_RADIUS, 2) + pow(DIST_REAR_AXLE_TO_FRONT, 2));
    double w     = m_LotDepth;
    double alpha = acos(1 - ((pow(DIST_REAR_AXLE_TO_FRONT, 2)) / (4*pow(MINIMAL_STEERING_RADIUS, 2))));
    double beta  = asin((MINIMAL_STEERING_RADIUS/R_e) * sin(alpha));
    double theta = acos((MINIMAL_STEERING_RADIUS - w) / R_e);

    ROS_INFO("re %f w %f alpha %f beta %f  theta %f", R_e, w, alpha, beta, theta);

    double degrees = radiansToDegrees(theta);
    ROS_INFO("Theta: %f", degrees);
    return theta;*/
} // CALCPARKING

