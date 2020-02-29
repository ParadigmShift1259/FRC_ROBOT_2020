 /**
 *  VIsion.cpp
 *  Date: 2/9/20
 *  Last Edited By: Geoffrey Xue
 */


#include "Vision.h"
#include "Const.h"


using namespace std;


 
Vision::Vision(OperatorInputs *inputs)
{
    m_inputs = inputs;
    m_networktable = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
    m_dashboard = nt::NetworkTableInstance::GetDefault().GetTable("SmartDashboard");
    m_camerachoice = 0;
    m_led = true;
}


Vision::~Vision()
{

}


void Vision::Init()
{
    m_tx = 0;
    m_ty = 0;
    m_ta = 0;
    m_ts = 0;
    m_active = false;

    m_verticalangle = 0;

    m_distance = 0;
    m_horizontalangle = 0;

    m_averagedistance[0] = 0;
    m_averagedistance[1] = 0;
    m_averagedistance[2] = 0;
    m_averageangle[0] = 0;
    m_averageangle[1] = 0;
    m_averagedistance[2] = 0;

    SetLED(true);
}


void Vision::Loop()
{
    m_dashboard->PutNumber("cameraFeed", m_camerachoice);

    m_active = m_networktable->GetNumber("tv", 0);

    if (!m_led)
        m_active = false;

    if (!m_active)
    {
        m_averagedistance[0] = 0;
        m_averagedistance[1] = 0;
        m_averagedistance[2] = 0;
        m_averageangle[0] = 0;
        m_averageangle[1] = 0;
        m_averagedistance[2] = 0;
        return;
    }

    m_tx = m_networktable->GetNumber("tx", 0.0);
    m_ty = m_networktable->GetNumber("ty", 0.0);

    /*
    m_ta = m_networktable->GetNumber("ta", 0.0);
    m_ts = m_networktable->GetNumber("ts", 0.0);
    m_tcornx = m_networktable->GetNumberArray("tcornx", 0.0);
    m_tcorny = m_networktable->GetNumberArray("tcorny", 0.0);
    */

    m_verticalangle = VIS_MOUNTING_ANGLE + m_ty;    // in degrees
    
    m_distance = (VIS_TARGET_HEIGHT - VIS_MOUNTING_HEIGHT) / tanf(DegreesToRadians(m_verticalangle));
    m_horizontalangle = m_tx;

    for (int i = 0; i < 2; i++)
    {
        m_averagedistance[i] = m_averagedistance[i + 1];
        m_averageangle[i] = m_averageangle[i + 1];
    }

    m_averagedistance[2] = m_distance;
    m_averageangle[2] = m_horizontalangle;

    SmartDashboard::PutNumber("VIS0_Active", m_active);
    SmartDashboard::PutNumber("VIS1_Distance", m_distance);
    SmartDashboard::PutNumber("VIS2_Angle", m_horizontalangle);
    SmartDashboard::PutNumber("VIS3_Average Distance", 
    (m_averagedistance[0] + m_averagedistance[1] + m_averagedistance[2]) / 3);
    SmartDashboard::PutNumber("VIS4_Average Angle", 
    (m_averageangle[0] + m_averageangle[1] + m_averageangle[2]) / 3);
}


void Vision::Stop()
{
    if (m_networktable != nullptr)
        SetLED(false);
}


bool Vision::GetActive()
{
    return m_active;
}


double Vision::GetDistance()
{
    return (m_averagedistance[0] + m_averagedistance[1] + m_averagedistance[2]) / 3;
}


double Vision::GetAngle()
{
    return m_horizontalangle;
}


void Vision::SetLED(bool on)
{
    m_led = on;
    if (m_led)
    {
        // 3 forces limelight led on
        m_networktable->PutNumber("ledMode", 3);
    }
    else
    {
        // 1 forces limelight led off
        m_networktable->PutNumber("ledMode", 1);
    }
}


double Vision::DegreesToRadians(double degrees)
{
    return degrees * 2 * 3.1415926535 / 360;
}


double Vision::RadiansToDegrees(double radians)
{
    return radians / 2 / 3.1415926535 * 360;
}


void Vision::IntakeSensorUpdate(bool update)
{
    m_dashboard->PutNumber("RollerSensor", update);
}


double Vision::IntakeDistance()
{
    return 0;
}


double Vision::IntakeAngle()
{
    return 0;
}