/**
 *  Drivetrain.h
 *  Date: 1/30/20
 *  Last Edited By: Geoffrey Xue
 */


#ifndef SRC_Drivetrain_H_
#define SRC_Drivetrain_H_

#include "OperatorInputs.h"

#include <ctre/Phoenix.h>
#include <frc/SpeedControllerGroup.h>
#include <frc/drive/DifferentialDrive.h>



using namespace std;
using namespace frc;


class Drivetrain
{
public:
	Drivetrain(OperatorInputs *inputs);
	~Drivetrain();
	void Init();
	void Loop();
	void Stop();
	void ReportData();
	void ResetEncoders();
	void ResetGyro();
	void ConfigureInverts();

	TalonFXSensorCollection *GetLeftSensor() {return m_leftsensor; }
	TalonFXSensorCollection *GetRightSensor() {return m_rightsensor; }
	PigeonIMU *GetGyro() {return m_gyro; }
	DifferentialDrive *GetDrive() {return m_differentialdrive; }


protected:
	OperatorInputs *m_inputs;
	WPI_TalonFX *m_left1;
	WPI_TalonFX *m_left2;
	WPI_TalonFX *m_left3;
	WPI_TalonFX *m_right1;
	WPI_TalonFX *m_right2;
	WPI_TalonFX *m_right3;
	SpeedControllerGroup *m_leftscgroup;
	SpeedControllerGroup *m_rightscgroup;
	DifferentialDrive *m_differentialdrive;
	TalonFXSensorCollection *m_leftsensor;
	TalonFXSensorCollection *m_rightsensor;
	PigeonIMU *m_gyro;

	bool m_leftinvert;
	bool m_rightinvert;

};


#endif /* SRC_Drivetrain_H_ */
