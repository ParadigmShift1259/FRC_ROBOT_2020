/**
 *  Autonomous.h
 *  Date:
 *  Last Edited By:
 */


#ifndef SRC_Autonomous_H_
#define SRC_Autonomous_H_

#include "OperatorInputs.h"
#include "GyroDrive.h"
#include "Intake.h"
#include "Feeder.h"
#include "Turret.h"
#include "Vision.h"

#include <frc/Timer.h>


using namespace frc;


class Autonomous
{
public:
	Autonomous(GyroDrive *gyrodrive, Intake *intake, Feeder *feeder, Turret *turret, Vision *vision);
	~Autonomous();
	void Init();
	void Loop();
	void Stop();

protected:
	void SimpleAuto();
    void DriveStraight();
    void TrenchRun();

protected:
    OperatorInputs *m_inputs;
    GyroDrive *m_gyrodrive;
    Intake *m_intake;
    Feeder *m_feeder;
    Turret *m_turret;
    Vision *m_vision;

    int m_stage;
	Timer m_timer;
};


#endif /* SRC_Autonomous_H_ */