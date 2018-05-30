// CPIDController.hpp

#ifndef __CPIDCONTROLLER_HPP
#define __CPIDCONTROLLER_HPP

// I N C L U D E S -------------------------------------------------------------------
#include <ros/ros.h>

// C L A S S - C P I D C O N T R O L L E R -------------------------------------------
class CPIDController {
public:

    CPIDController(double dkp, double dki, double dkd, double ddt, double dMax, double dMin) :
      m_dkp(dkp),
      m_dkd(dkd),
      m_dki(dki),
      m_ddt(ddt),
      m_dMax(dMax),
      m_dMin(dMin),
      m_dPreError(0),
      m_dIntegral(0)
  {};

    double compute(double dSetpoint, double dMeasure);
    void reset();
    void setDt(double dt);

private:
    double m_dkp;
    double m_dkd;
    double m_dki;
    double m_ddt;
    double m_dMax;
    double m_dMin;
    double m_dPreError;
    double m_dIntegral;
    double m_LastOutput;
}; // CLASS CPIDCONTROLLER

#endif // __CPIDCONTROLLER_HPP
