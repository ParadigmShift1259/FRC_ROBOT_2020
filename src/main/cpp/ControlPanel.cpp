/**
 *  ControlPanel.cpp
 *  Date: 1/7/2020
 *  Last Edited By: Jival Chandrashekar
 */


#include "ControlPanel.h"
#include "Const.h"


using namespace std;


ControlPanel::ControlPanel(OperatorInputs *inputs)
{
    m_inputs = inputs;
    static constexpr auto i2cPort = frc::I2C::Port::kOnboard;
    m_colorsensor = new rev::ColorSensorV3(i2cPort);
    m_spinner = new TalonSRX(0);

    m_spinnerstate = kOff;

    m_spinpower = 0;
    m_desiredcolor = 0; // 1-4 Yellow, Green, Blue, Red
    m_currentcolor = 0;
    m_previouscolor = 0;
}


ControlPanel::~ControlPanel()
{
    if (m_colorsensor != nullptr)
        delete m_colorsensor;
    if (m_spinner != nullptr)
        delete m_spinner;
}


void ControlPanel::Init()
{
    m_spinner->SetNeutralMode(NeutralMode::Coast);
    m_spinner->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, TUR_TIMEOUT_MS);
    m_spinner->Set(ControlMode::PercentOutput, 0);
    m_spinner->SetSensorPhase(true);
    m_spinner->SetInverted(true);

    m_spinnerstate = kOff;

    m_spinpower = 0;
    m_desiredcolor = 0; // 1-4 Yellow/Red/Green/Blue
    m_currentcolor = 0;
    m_colorcounter = 0;
    m_registeredcolor = 0;
    m_colorregistertimes = 0;
    m_stop = false;
    SmartDashboard::PutNumber("CS11_SpinPower", m_spinpower);
    SmartDashboard::PutNumber("CS12_DesiredColor(YRGB)", m_desiredcolor);
    SmartDashboard::PutNumber("CS13_ColorsCounter", m_colorcounter);
    SmartDashboard::PutBoolean("Again", true);
    
}


void ControlPanel::Loop()
{
    switch (m_spinnerstate)
    {
        case kOff:

            m_currentcolor = GetColor();
            if (m_currentcolor == m_previouscolor && m_registeredcolor != m_currentcolor)
            {
                m_colorcount++;
            }
            else
            {
                m_previouscolor = m_currentcolor;
            }
            
            if (m_colorcount >= 2)
            {
                if (m_currentcolor == 1)
                {
                    SmartDashboard::PutString("CS100_TrueColor", "Yellow");
                    m_registeredcolor = 1;
                }
                else if (m_currentcolor == 2)
                {
                    SmartDashboard::PutString("CS100_TrueColor", "Red");
                    m_registeredcolor = 2;
                }
                else if (m_currentcolor == 3)
                {
                    SmartDashboard::PutString("CS100_TrueColor", "Green");
                    m_registeredcolor = 3;
                }
                else if (m_currentcolor == 4)
                {
                    SmartDashboard::PutString("CS100_TrueColor", "Blue");
                    m_registeredcolor = 4;
                    m_colorregistertimes++;
                }
                m_previouscolor = m_currentcolor;
                m_colorcount = 0;          
            }

            if (m_colorregistertimes >= 7)
                m_stop = true;
            
            if (m_stop)
            {
                if (m_inputs->xBoxAButton(OperatorInputs::ToggleChoice::kToggle, 0))
                {
                    m_stop = false;
                    m_colorregistertimes = 0;
                }
                m_spinner->Set(ControlMode::PercentOutput, 0);
            }
            else
                m_spinner->Set(ControlMode::PercentOutput, m_spinpower);     
            break;
        case kBlindSpin:
            /*SmartDashboard::PutNumber("CS14_Proximity", m_colorsensor->GetProximity());
            m_spinner->Set(ControlMode::PercentOutput, m_spinpower);

            if (GetColor() == m_desiredcolor && m_desiredcolor == m_previouscolors[0])
            {
                m_spinnerstate = kOff;
                m_spinner->Set(ControlMode::PercentOutput, 0);
            }
            
            m_previouscolors[0] = GetColor();*/
            break;
        case kColorSpin:

            m_spinner->Set(ControlMode::PercentOutput, m_spinpower);
            break;
    }

    m_spinpower = SmartDashboard::GetNumber("CS11_SpinPower", 0);
    m_desiredcolor = SmartDashboard::GetNumber("CS12_DesiredColor(YRGB)", 0);
    SmartDashboard::PutNumber("CS13_ColorsCounter", m_colorcount);
}


void ControlPanel::Stop()
{
    m_spinnerstate = kOff;
}


int ControlPanel::GetColor()
{
    double IR = m_colorsensor->GetIR();
    frc::Color detectedColor = m_colorsensor->GetColor();
    uint32_t proximity = m_colorsensor->GetProximity();
    double K = 1 - fmax(fmax(detectedColor.red, detectedColor.blue), detectedColor.green);
    double C = (1 - detectedColor.red - K) / (1 - K);
    double M = (1 - detectedColor.green - K) / (1 - K);
    double Y = (1 - detectedColor.blue - K) / (1 - K);
    
    int color = 0; // 1-4 Yellow, Green, Blue, Red

    if (C < .300)
    {
        color = 2;
        SmartDashboard::PutString("CSTEST_Color", "Red");
    }
    else if (Y < .300)
    {
        color = 4;
        SmartDashboard::PutString("CSTEST_Color", "Blue");
    }
    else if (M < .200  && C > Y )
    {
        color = 3;
        SmartDashboard::PutString("CSTEST_Color", "Green");
    }
    else if (M < .200  && Y > C )
    {
        color = 1;
        SmartDashboard::PutString("CSTEST_Color", "Yellow");
    }

    SmartDashboard::PutNumber("CS1_K", K );
    SmartDashboard::PutNumber("CS2_C", C);
    SmartDashboard::PutNumber("CS3_M", M);
    SmartDashboard::PutNumber("CS4_Y", Y);
    SmartDashboard::PutNumber("CS5_IR", IR);
    SmartDashboard::PutNumber("CS6_Proximity",proximity);
    SmartDashboard::PutNumber("CS7_Color",color);
    SmartDashboard::PutNumber("CS9_Encoder_Position in Revolutions", m_spinner->GetSelectedSensorPosition(0) / ENCODER_TICKS_PER_REV);
    SmartDashboard::PutNumber("CS10_Encoder_Velocity in RPM", m_spinner->GetSelectedSensorVelocity(0) / MINUTES_TO_HUNDRED_MS / ENCODER_TICKS_PER_REV);
    SmartDashboard::PutNumber("CS11_Motor_Output", m_spinner->GetMotorOutputPercent());

    return color;
}