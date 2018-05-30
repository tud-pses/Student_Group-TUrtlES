// CPIDController.cpp

// I N C L U D E S -------------------------------------------------------------------
#include <limits.h>

// O W N - I N C L U D E S -----------------------------------------------------------
#include "CPIDController.hpp"

// C L A S S - C P I D C O N T R O L L E R -------------------------------------------
double CPIDController::compute(double dSetpoint, double dMeasure) {
    double error      = dSetpoint - dMeasure;
    double derivative = (error - m_dPreError) / m_ddt;

    double output = error*m_dkp + m_dIntegral*m_dki + m_dkd*derivative;

    // Anti-windup
    if (fabs(output) >= DBL_MAX && (((error >= 0) && (m_dIntegral >= 0)) || ((error < 0) && (m_dIntegral < 0))))
        m_dIntegral = m_dIntegral;
    else
        m_dIntegral += error*m_ddt;

    m_dPreError = error;

    if (output >= m_dMax)
        output = m_dMax;
    else if (output <= m_dMin)
        output = m_dMin;

    return output;
} // COMPUTE

void CPIDController::reset() {
    m_dPreError = 0;
    m_dIntegral = 0;
} // RESET

void CPIDController::setDt(double dt) {
    m_ddt = dt;
}
