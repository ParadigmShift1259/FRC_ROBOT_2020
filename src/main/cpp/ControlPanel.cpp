/**
 *  ControlPanel.cpp
 *  Date: 1/7/2020
 *  Last Edited By: Jival Chandrashekar
 */


#include "ControlPanel.h"
#include "Const.h"


using namespace std;


static constexpr Color kYellowTarget = Color(.323364, .588501, .088013);
static constexpr Color kRedTarget = Color(0.580933, 0.310425, 0.108521);    
static constexpr Color kGreenTarget = Color(0.118286, 0.574829, .302368);
static constexpr Color kBlueTarget = Color(0.084595, 0.361450, 0.553833);


ControlPanel::ControlPanel(OperatorInputs *inputs)
{
    m_inputs = inputs;
    static constexpr auto i2cPort = frc::I2C::Port::kOnboard;
    m_colorsensor = new rev::ColorSensorV3(i2cPort);
    m_spinner = new TalonSRX(0);

    m_spinnerstate = kOff;

    m_currentcolor = 0;
    m_previouscolor = 0;
    m_registeredcolor = 0;
    m_colorbouncecount = 0;
    m_colorregisteredcount[0] = 0;
    m_colorregisteredcount[1] = 0;
    m_colorregisteredcount[2] = 0;
    m_colorregisteredcount[3] = 0;
    m_stop = false;
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
    m_spinpower = 0;

    m_spinnerstate = kBlindSpin;

    m_colormatcher.AddColorMatch(kYellowTarget);
    m_colormatcher.AddColorMatch(kRedTarget);
    m_colormatcher.AddColorMatch(kGreenTarget);
    m_colormatcher.AddColorMatch(kBlueTarget);
    m_confidence = 0.6;

    m_currentcolor = 0;
    m_previouscolor = 0;
    m_registeredcolor = 0;
    m_colorbouncecount = 0;
    m_colorregisteredcount[0] = 0;
    m_colorregisteredcount[1] = 0;
    m_colorregisteredcount[2] = 0;
    m_colorregisteredcount[3] = 0;
    m_stop = false;

    SmartDashboard::PutNumber("CS11_SpinPower", m_spinpower);
    SmartDashboard::PutNumber("CS19_Confidence", m_confidence);
}


void ControlPanel::Loop()
{
    switch (m_spinnerstate)
    {
        case kOff:   
            break;
        case kBlindSpin:
            /**
             * Variables used:
             * m_currentcolor
             * m_previouscolor
             * m_registeredcolor
             * m_colorbouncecount
             * m_colorregisteredcount
             * m_stop
             * 
             * m_spinner
             * m_inputs
             * 
             * m_spinpower
            */
            m_currentcolor = GetColor();
            // Start bounce counting if:
            // Color is picked up twice
            // Color is not the same color as the registered one already
            // 
            // Color must pass filter that checks for sensor inconsistencies
            if (m_currentcolor == m_previouscolor && m_registeredcolor != m_currentcolor && SensorSanityCheck())
            {
                m_colorbouncecount++;
            }
            else if (SensorSanityCheck())
            {
                m_previouscolor = m_currentcolor;
            }
            // Once picked up twice, set and log the registered color and reset color bounce count
            if (m_colorbouncecount >= 2)
            {
                if (m_currentcolor == 1)
                {
                    SmartDashboard::PutString("CS100_TrueColor", "Yellow");
                    m_registeredcolor = 1;
                    m_colorregisteredcount[0]++;
                }
                else if (m_currentcolor == 2)
                {
                    SmartDashboard::PutString("CS100_TrueColor", "Red");
                    m_registeredcolor = 2;
                    m_colorregisteredcount[1]++;
                }
                else if (m_currentcolor == 3)
                {
                    SmartDashboard::PutString("CS100_TrueColor", "Green");
                    m_registeredcolor = 3;
                    m_colorregisteredcount[2]++;
                }
                else if (m_currentcolor == 4)
                {
                    SmartDashboard::PutString("CS100_TrueColor", "Blue");
                    m_registeredcolor = 4;
                    m_colorregisteredcount[3]++;
                }
                m_previouscolor = m_currentcolor;
                m_colorbouncecount = 0;          
            }

            // If one color is registered enough times, stop spinning
            if (m_colorregisteredcount[3] >= 7)
                m_stop = true;
            
            // Once stopped, use A button to restart. If not, set the speed to spinpower
            if (m_stop)
            {
                if (m_inputs->xBoxAButton(OperatorInputs::ToggleChoice::kToggle, 0))
                {
                    m_stop = false;
                    m_colorregisteredcount[0] = 0;
                    m_colorregisteredcount[1] = 0;
                    m_colorregisteredcount[2] = 0;
                    m_colorregisteredcount[3] = 0;
                }
                m_spinner->Set(ControlMode::PercentOutput, 0);
            }
            else
            {
                m_spinner->Set(ControlMode::PercentOutput, m_spinpower); 
            }

            if (m_inputs->xBoxBButton(OperatorInputs::ToggleChoice::kToggle, 0))
            {
                m_colorregisteredcount[0] = 0;
                m_colorregisteredcount[1] = 0;
                m_colorregisteredcount[2] = 0;
                m_colorregisteredcount[3] = 0;
            }
            break;
    
        case kColorSpin:
            m_spinner->Set(ControlMode::PercentOutput, m_spinpower);
            break;
    }

    m_spinpower = SmartDashboard::GetNumber("CS11_SpinPower", 0);
    m_confidence = SmartDashboard::GetNumber("CS19_Confidence", 0);

    // Displays the total registered color count of each color
    SmartDashboard::PutNumber("CS20_Registered Yellow Count", m_colorregisteredcount[0]);
    SmartDashboard::PutNumber("CS30_Registered Red Count", m_colorregisteredcount[1]);
    SmartDashboard::PutNumber("CS40_Registered Green Count", m_colorregisteredcount[2]);
    SmartDashboard::PutNumber("CS50_Registered Blue Count", m_colorregisteredcount[3]);

    // Motor data
    SmartDashboard::PutNumber("CS9_Encoder_Position in Revolutions", m_spinner->GetSelectedSensorPosition(0) / ENCODER_TICKS_PER_REV);
    SmartDashboard::PutNumber("CS10_Encoder_Velocity in RPM", m_spinner->GetSelectedSensorVelocity(0) / MINUTES_TO_HUNDRED_MS / ENCODER_TICKS_PER_REV);
    SmartDashboard::PutNumber("CS11_Motor_Output", m_spinner->GetMotorOutputPercent());

}


void ControlPanel::Stop()
{
    m_spinnerstate = kOff;
}

/*
int ControlPanel::GetColor()
{
    //double IR = m_colorsensor->GetIR();
    Color detectedColor = m_colorsensor->GetColor();
    //uint32_t proximity = m_colorsensor->GetProximity();
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

    SmartDashboard::PutNumber("CS1_K", K);
    SmartDashboard::PutNumber("CS2_C", C);
    SmartDashboard::PutNumber("CS3_M", M);
    SmartDashboard::PutNumber("CS4_Y", Y);
    SmartDashboard::PutNumber("CS1000_R", detectedColor.red);
    SmartDashboard::PutNumber("CS2000_G", detectedColor.green);
    SmartDashboard::PutNumber("CS3000_B", detectedColor.blue);
    //SmartDashboard::PutNumber("CS5_IR", IR);
    //SmartDashboard::PutNumber("CS6_Proximity",proximity);
    SmartDashboard::PutNumber("CS7_Color", color);
    return color;
}
*/

int ControlPanel::GetColor()
{
    Color detectedcolor = m_colorsensor->GetColor();
    Color matchedcolor = m_colormatcher.MatchClosestColor(detectedcolor, m_confidence);

    int color = 0;

    if (matchedcolor == kYellowTarget)
        color = 1;
    else 
    if (matchedcolor == kRedTarget)
        color = 2;
    else
    if (matchedcolor == kGreenTarget)
        color = 3;
    else
    if (matchedcolor == kBlueTarget)
        color = 4;

    SmartDashboard::PutNumber("CS1000_R", detectedcolor.red);
    SmartDashboard::PutNumber("CS2000_G", detectedcolor.green);
    SmartDashboard::PutNumber("CS3000_B", detectedcolor.blue);
    SmartDashboard::PutNumber("CS7_Color", color);

    return color;
}

bool ControlPanel::SensorSanityCheck()
// Checks if it has gathered enough data
// Once it has, check for color mis-followings with the color sensor
//      For example, if a yellow shows up after a green even though the pattern is YRGB
{
    // if sensor has not gotten enough data yet, cancel out of sanity check
    for (int i = 0; i < 4; i++)
    {
        if (m_colorregisteredcount[i] < 1)
            return true;
    }

    if (m_currentcolor == 0 || m_previouscolor == 0)
        return true;

    // color can never be two states ahead/below itself
    double impossiblecolor1 = m_previouscolor - 1;

    // rescaling back to 1 - 4 bounds
    if (impossiblecolor1 == 0)
        impossiblecolor1 = 4;

    double impossiblecolor2 = m_previouscolor + 2;

    if (impossiblecolor2 == 5)
        impossiblecolor2 = 1;
    else
    if (impossiblecolor2 == 6)
        impossiblecolor2 = 2;
    
    
    if (m_currentcolor == impossiblecolor1 || m_currentcolor == impossiblecolor2)
        return false;
    else
        return true;
}