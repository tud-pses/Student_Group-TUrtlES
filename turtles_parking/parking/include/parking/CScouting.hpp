// CScouting.hpp

#ifndef __CSCOUTING_HPP
#define __CSCOUTING_HPP

// I N C L U D E S -------------------------------------------------------------------
#include <functional>
#include <memory>
#include <vector>
#include <math.h>
#include <ros/ros.h>

// O W N - I N C L U D E S -----------------------------------------------------------
#include "CSensors.hpp"
#include "CMovement.hpp"
#include "CHelpers.hpp"
#include "CPIDController.hpp"

// T Y P E S  A N D  E N U M S ------------------------------------------------------
typedef std::function<void(SParkingLot p)> CScoutingCB;

enum SStatus { SS_FIND_WALL,            // Find a wall and lock to it
               SS_FOLLOW_WALL,          // Follow the wall with angle theta and distance dist
               SS_PREPARE_FOR_PARKING,  // Calc parking-params
               SS_FINISHED              // Scouting finished
}; // ENUM PSTATUS

// C L A S S - C S C O U T I N G ----------------------------------------------------
class CScouting {
public:
    CScouting(CSensors &sensors, CMovement &movement, CScoutingCB callback);

    PCB spin();
  
private:
    void   findWall();
    void   followWall();
    void   registerBox(double delta);
    double calcTheta();
    void   prepareForParking();

    CSensors  &m_Sensors;
    CMovement &m_Movement;

    std::unique_ptr<CPIDController> m_distController;

    SStatus m_Status;
    int     m_WallFoundCnt;
    bool    m_FirstBox;
    double  m_LastBox;
    double  m_TargetDistance;
    double  m_DrivenDist;
    double  m_ParkingLotWidth;
    double  m_LotDepth;
    int     m_PIDCnt;

    CScoutingCB m_callback;
}; // CLASS CSCOUTING

#endif // __CSCOUTING_HPP
