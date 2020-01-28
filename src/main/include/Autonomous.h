/**
 *  Autonomous.h
 *  Date:
 *  Last Edited By:
 */


#ifndef SRC_Autonomous_H_
#define SRC_Autonomous_H_


#include <ctre\Phoenix.h>
#include "OperatorInputs.h"
#include "DriveTrainFX.h"

// Ramsete Controller Implementation

#include <units/units.h>
#include <frc2/command/RamseteCommand.h>
#include <frc/controller/RamseteController.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>


using namespace frc;


class Autonomous
{
public:
	Autonomous(OperatorInputs *inputs);
	~Autonomous();
	void Init();
	void Loop();
	void Stop();

protected:
    OperatorInputs *m_inputs;
};


#endif /* SRC_Autonomous_H_ */
