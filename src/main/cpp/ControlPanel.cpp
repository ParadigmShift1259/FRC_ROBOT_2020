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
}


void ControlPanel::Loop()
{
    double IR = m_colorsensor->GetIR();
    frc::Color detectedColor = m_colorsensor->GetColor();
    uint32_t proximity = m_colorsensor->GetProximity();
    double K = 1 - fmax(fmax(detectedColor.red, detectedColor.blue), detectedColor.green);
    double C = (1 - detectedColor.red - K) / (1 - K);
    double M = (1 - detectedColor.green - K) / (1 - K);
    double Y = (1 - detectedColor.blue - K) / (1 - K);
    
    int color = 0; // 1-4 Yellow, Green, Blue, Red

    if (C < .201)
        color = 4;
    else if (Y < .201)
        color = 3;
    else if (M < .200  && C > Y )
        color = 2;
    else if (M < .200  && Y > C )
        color = 1;
    
    m_spinner->Set(ControlMode::PercentOutput, m_inputs->xBoxLeftY(0));

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
}


void ControlPanel::Stop()
{
}
