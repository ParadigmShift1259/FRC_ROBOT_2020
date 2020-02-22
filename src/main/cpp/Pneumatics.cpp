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
	m_log = g_log;
	m_compressor = nullptr;
	m_pdp = nullptr;
	m_pressureSwitch = false;
	m_compressorCurrent = 0;
	m_totalCurrent = 0;
	m_voltage = 0;
	m_temperature = 0;

	if (PCM_COMPRESSOR_SOLENOID != -1)
		m_compressor = new Compressor(PCM_COMPRESSOR_SOLENOID);

	if (CAN_POWER_DISTRIBUTION_PANEL != -1)
		m_pdp = new PowerDistributionPanel(CAN_POWER_DISTRIBUTION_PANEL);
	
	m_stage = kRun;

	m_log->logMsg(eInfo, __FUNCTION__, __LINE__, "stage,pressureSwitch,compressorCurrent,totalCurrent,voltage,temperature");
	m_dataInt.push_back((int*)&m_stage);
	m_dataInt.push_back((int*)&m_pressureSwitch);
	
	m_dataDouble.push_back(&m_compressorCurrent);
	m_dataDouble.push_back(&m_totalCurrent);
	m_dataDouble.push_back(&m_voltage);
	m_dataDouble.push_back(&m_temperature);
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

	m_pressureSwitch = m_compressor->GetPressureSwitchValue();
	m_totalCurrent = m_pdp->GetTotalCurrent();
	m_voltage = m_pdp->GetVoltage();
	m_compressorCurrent = m_compressor->GetCompressorCurrent();
	m_temperature = m_pdp->GetTemperature();

	switch (m_stage)
	{
	case kRun:
		if ((m_totalCurrent > PNE_CURRENT_DRAW) ||
			(m_voltage < PNE_VOLTAGE_DROP))
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

	m_log->logData(__FUNCTION__, __LINE__, m_dataInt, m_dataDouble);
}


void Pneumatics::Stop()
{
	if (m_compressor != nullptr)
		m_compressor->Stop();
}
