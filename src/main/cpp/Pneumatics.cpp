/**
 *  Pneumatics.cpp
 *  Date:
 *  Last Edited By:
 */


#include "Pneumatics.h"
#include "Const.h"


using namespace std;


Pneumatics::Pneumatics()
{
	m_compressor = nullptr;
    m_pdp = nullptr;

	if (PCM_COMPRESSOR_SOLENOID != -1)
		m_compressor = new Compressor(PCM_COMPRESSOR_SOLENOID);

    if (CAN_POWER_DISTRIBUTION_PANEL != -1)
        m_pdp = new PowerDistributionPanel(CAN_POWER_DISTRIBUTION_PANEL);
    
    m_stage = kRun;
}


Pneumatics::~Pneumatics()
{
    if (m_compressor != nullptr)
        delete m_compressor;
    if (m_pdp != nullptr)
        delete m_pdp;
}


void Pneumatics::Init()
{
	if (m_compressor != nullptr)
    {
        m_compressor->ClearAllPCMStickyFaults();
		m_compressor->Start();
    }
    if (m_pdp != nullptr)
    {
        m_pdp->ClearStickyFaults();
    }
    
    m_stage = kRun;
    m_timer.Start();
}


void Pneumatics::Loop()
{
    if ((m_compressor == nullptr) || (m_pdp == nullptr))
        return;
    
    switch (m_stage)
    {
    case kRun:
        if ((m_pdp->GetTotalCurrent() > PNE_CURRENT_DRAW) ||
            (m_pdp->GetVoltage() < PNE_VOLTAGE_DROP))
        {
            m_compressor->Stop();
            m_timer.Reset();
            m_stage = kWait;
        }
        else
        {
            m_compressor->Start();
        }
        break;

    case kWait:
        if (m_timer.Get() > PNE_WAITTIME)
        {
            m_stage = kRun;
        }
        break;
    }
}


void Pneumatics::Stop()
{
	if (m_compressor != nullptr)
		m_compressor->Stop();
}
