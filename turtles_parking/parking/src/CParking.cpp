// CParking.cpp

// I N C L U D E S -------------------------------------------------------------------
#include "CParking.hpp"

// C L A S S - C P A R K I N G -------------------------------------------------------
CParking::CParking(ros::NodeHandle &nh) : m_nh(nh) {
    m_Status = PS_SCOUTING;

    std::unique_ptr<CSensors> sensors(new CSensors(nh));
    std::unique_ptr<CMovement> movement(new CMovement(nh));

    using std::placeholders::_1;
    std::unique_ptr<CScouting> scouting(new CScouting(
          *sensors,
          *movement,
          std::bind(&CParking::didFindParkingLot, this, _1)
          ));

    std::unique_ptr<CParkProcess> pprocess(new CParkProcess(
          *sensors,
          *movement,
          std::bind(&CParking::didFinishParking, this, _1)
          ));

    m_Sensors     = std::move(sensors);
    m_Movement    = std::move(movement);
    m_Scouting    = std::move(scouting);
    m_ParkProcess = std::move(pprocess);
} // CONSTRUCTOR

PCB CParking::spin() {
    switch(m_Status) {
        case PS_SCOUTING: {
            checkForEmergency();
            return m_Scouting->spin();
        } // CASE PS_SCOUTING
        case PS_PARKING: {
          return m_ParkProcess->spin();
        } break; // CASE PS_PARKING
        case PS_FINISHED: {
            return PCB_PARKING_FINISHED;
        } // CASE PS_FINISHED
        case PS_EMERGENCY: {
            checkForEmergency();
            return emergencyStop();
        }
    } // SWITCH

    return PCB_OK;
} // SPIN

void CParking::checkForEmergency() {
    double front = m_Sensors->getDistFront();
    if (front > EMERGENCY_FRONT_TRESHOLD) {
      if (m_Status == PS_EMERGENCY) {
        m_Movement->setMotorSpeed(m_MotorSpeedBeforeEmergency);
        m_Status = m_StatusBeforeEmergency;
      }
    } else {
      if (m_Status != PS_EMERGENCY) {
        m_MotorSpeedBeforeEmergency = m_Movement->getMotorSpeed();
        m_StatusBeforeEmergency = m_Status;
        m_Movement->setMotorSpeed(0);
        m_Status = PS_EMERGENCY;
        m_EmergencyTicks = 0;
      }
    }
}

PCB CParking::emergencyStop() {
  if (m_EmergencyTicks > EMERGENCY_STOP_TICKS) {
    return PCB_ERROR;
  }
  m_EmergencyTicks++;
  return PCB_OK;
}

/**
 * This is the CScouting callblack
 */
void CParking::didFindParkingLot(SParkingLot plot) {
    m_ParkProcess->setParkingLot(plot);
    m_Status = PS_PARKING;
} // DIDFINDPARKINGLOT

/**
 * Parking process callback
 */
void CParking::didFinishParking(bool success) {
    if(success)
      m_Status = PS_SCOUTING;
    else
      m_Status = PS_FINISHED;
} // DIDFINISHPAPRKING

