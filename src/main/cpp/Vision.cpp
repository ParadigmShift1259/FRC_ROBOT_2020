 /**
 *  VIsion.cpp
 *  Date: 2/9/20
 *  Last Edited By: Geoffrey Xue
 */


#include "Vision.h"
#include "Const.h"


using namespace std;


 
Vision::Vision()
{
    m_networktable = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
}


Vision::~Vision()
{

}


void Vision::Init()
{

}


void Vision::Loop()
{
    m_active = m_networktable->GetNumber("tv", 0);
    if (!m_active)
        return;

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

    SmartDashboard::PutNumber("VIS1_Distance", m_distance);
    SmartDashboard::PutNumber("VIS2_Angle", m_horizontalangle);
}


void Vision::Stop()
{

}


bool Vision::GetActive()
{
    return m_active;
}


double Vision::GetDistance()
{
    return m_distance;
}


double Vision::GetAngle()
{
    return m_horizontalangle;
}


double Vision::DegreesToRadians(double degrees)
{
    return degrees * 2 * 3.1415926535 / 360;
}


double Vision::RadiansToDegrees(double radians)
{
    return radians / 2 / 3.1415926535 * 360;
}