// CParkProcess.hpp

#ifndef __CPARKPROCESS_HPP
#define __CPARKPROCESS_HPP

// I N C L U D E S ------------------------------------------------------------------
#include <functional>

// O W N - I N C L U D E S -----------------------------------------------------------
#include "CSensors.hpp"
#include "CMovement.hpp"
#include "CHelpers.hpp"
#include "CPIDController.hpp"

// E N U M S ------------------------------------------------------------------------
enum PPStatus { PPS_I_DRIVE_FIRST_CIRCLE, // Prepare for driving first circle (init)
                PPS_DRIVE_FIRST_CIRCLE,   // Drive first circle
                PPS_I_DRIVE_SEC_CIRCLE,   // Prepare for driving second circle (init))
                PPS_DRIVE_SEC_CIRCLE,     // Drive second circle
                PPS_FINISH,               // Finish the parking_process
                PPS_DRIVE_FORWARD         // Tries to stand in the middle of the lot
}; // ENUM PPSTATUS

// T Y P E D E F S ------------------------------------------------------------------
typedef std::function<void(bool success)> CParkProcessCB;

// C L A S S - C P A R K P R O C E S S ----------------------------------------------
class CParkProcess {
public:
    CParkProcess(CSensors &sensors, CMovement &movement, CParkProcessCB callback);
    ~CParkProcess();

    PCB spin();
    void setParkingLot(SParkingLot parkingLot);

private:
    CSensors  &m_Sensors;
    CMovement &m_Movement;

    std::unique_ptr<CPIDController> m_distController;

    PPStatus    m_Status;
    SParkingLot m_ParkingLot;
    double      m_DrivenDist;
    double      m_InLotDistance;
    double      m_FrontGoalDist;

    CParkProcessCB m_callback;
}; // CLASS CPARKPROCESS

#endif // __CPARKPROCESS_HPP
