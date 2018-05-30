// CParkProcess.cpp

// I N C L U D E S ------------------------------------------------------------------
#include "CParkProcess.hpp"

// C L A S S - C P A R K P R O C E S S ----------------------------------------------
CParkProcess::CParkProcess(CSensors &sensors, CMovement &movement, CParkProcessCB callback) :
    m_Sensors(sensors),
    m_Movement(movement),
    m_callback(callback) {

    m_Status        = PPS_I_DRIVE_FIRST_CIRCLE;
    m_DrivenDist    = 0.0;
    m_InLotDistance = 0.0;
    m_FrontGoalDist = 0.0;

    std::unique_ptr<CPIDController> controller(new CPIDController(
          PID_P, PID_I, PID_D, 1.0/PARKING_LOOP_RATE, MAX_SCOUTING_STEERING, -MAX_SCOUTING_STEERING));
    m_distController = std::move(controller);
} // CONSTRUCTOR

CParkProcess::~CParkProcess() {
    return;
} // DESTRUCTOR


void CParkProcess::setParkingLot(SParkingLot parkingLot) {
    m_ParkingLot = parkingLot;
} // SETPARKINGLOT

PCB CParkProcess::spin() {
    switch (m_Status) {
        case PPS_I_DRIVE_FIRST_CIRCLE: {
            ROS_INFO("  ParkProcess: Starting park-process");

            m_Sensors.startTrackingYaw();

            m_Movement.setSteeringLevel(STEER_FOR_MIN_RAD_RIGHT);
            m_Movement.setMotorSpeed(-PARKING_SPEED);

            m_Status = PPS_DRIVE_FIRST_CIRCLE;

            break;
        } // CASE PPS_I_DRIVE_FIRST_CIRCLE
        case PPS_DRIVE_FIRST_CIRCLE: {
            // Check with odometry if car has turned around theta degrees
            if (m_Sensors.getCurrentYaw() >= m_ParkingLot.fTheta) {
                m_Status = PPS_I_DRIVE_SEC_CIRCLE;
                m_Movement.setMotorSpeed(0);
            } // IF

            break;
        } // CASE PPS_DRIVE_FIRST_CIRCLE
        case PPS_I_DRIVE_SEC_CIRCLE: {
            m_Sensors.startTrackingYaw();

            m_Movement.setSteeringLevel(STEER_FOR_MIN_RAD_LEFT);
            m_Movement.setMotorSpeed(-PARKING_SPEED);

            m_Status = PPS_DRIVE_SEC_CIRCLE;

            break;
        } // PPS_DRIVE_I_SEC_CIRCLE
        case PPS_DRIVE_SEC_CIRCLE: {
            // Check with odometry if car has turned around theta/2 degrees
            if (m_Sensors.getCurrentYaw() <= -(m_ParkingLot.fTheta/2.0)) {
                m_Status        = PPS_DRIVE_FORWARD;
                m_InLotDistance = m_Sensors.getDistRight();
                m_FrontGoalDist = ((m_ParkingLot.fPLWidth - CAR_LENGTH) / 2.0);
                m_Sensors.stopTrackingYaw();
                m_Movement.setMotorSpeed(PARKING_SPEED);
                m_Movement.setSteeringLevel(0);
                //ROS_INFO("  ParkProcess: Try to park in the middle of the lot %f auto %f goeal %f", m_ParkingLot.fPLWidth, CAR_LENGTH, m_FrontGoalDist);
            } // IF

            break;
        } // CASE PPS_DRIVE_SEC_CIRCLE
        case PPS_DRIVE_FORWARD: {
            // Try to follow wall in the lot
            /**/
            double distance = m_Sensors.getDistRight();
            double steering = 0.0;
            steering = m_distController->compute(0, (distance - m_InLotDistance)*100);
            m_Movement.setSteeringLevel(-steering);

            // Try to drive to the middle of the lot
            if (m_Sensors.getDistFront() <= m_FrontGoalDist) {
                m_Status = PPS_FINISH;
                m_Movement.setMotorSpeed(0);
                m_Movement.setSteeringLevel(0);
            } // IF

            break;
        } // PPS_DRIVE_FORWARD
        case PPS_FINISH: {
            ROS_INFO("  ParkProcess: Finished Parking");
            m_callback(true);

            return PCB_PARKING_FINISHED;
        } // CASE PPS_FINISH
    } // SWITCH

    return PCB_OK;
} // SPIN
