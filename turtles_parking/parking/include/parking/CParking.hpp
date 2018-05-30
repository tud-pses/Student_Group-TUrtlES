// CParking.hpp

#ifndef __CPARKING_HPP
#define __CPARKING_HPP

// I N C L U D E S -------------------------------------------------------------------
#include <functional>
#include <memory>

// O W N - I N C L U D E S -----------------------------------------------------------
#include "CScouting.hpp"
#include "CMovement.hpp"
#include "CSensors.hpp"
#include "CParkProcess.hpp"
#include "CHelpers.hpp"

// E N U M S -------------------------------------------------------------------------
enum PStatus { PS_SCOUTING,               // Search for parking-lot
               PS_PARKING,                // Parking in the found and measured parking-lot based on computed params
               PS_FINISHED,                // Robot has finished parking
               PS_EMERGENCY
}; // ENUM PSTATUS

// C L A S S - P A R K I N G ---------------------------------------------------------
class CParking {
public:
    // Constructor and Destructor
    CParking(ros::NodeHandle &nh);

    // Call to start sequence
    PCB spin();

    // CScouting callback
    void didFindParkingLot(SParkingLot pLot);

    // CParkProcess callback
    void didFinishParking(bool success);


private:
    // Emergency stop if object ahead
    PCB emergencyStop();
    void checkForEmergency();

    ros::NodeHandle &m_nh;
    PStatus       m_Status;

    std::unique_ptr<CScouting> m_Scouting;
    std::unique_ptr<CMovement> m_Movement;
    std::unique_ptr<CSensors> m_Sensors;
    std::unique_ptr<CParkProcess> m_ParkProcess;

    int m_MotorSpeedBeforeEmergency;
    int m_EmergencyTicks;
    PStatus m_StatusBeforeEmergency;
}; // CLASS CPARKING

#endif // __CPARKING_HPP
