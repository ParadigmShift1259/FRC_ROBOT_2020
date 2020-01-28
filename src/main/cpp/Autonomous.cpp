/**
 *  Autonomous.cpp
 *  Date:
 *  Last Edited By:
 */

#include "Autonomous.h"

Autonomous::Autonomous(OperatorInputs *inputs)
{
    m_inputs = inputs;

}


Autonomous::~Autonomous()
{

}


void Autonomous::Init()
{
  TrajectoryConfig config(1_mps, 1_mps / 1_s);

  // An example trajectory to follow.  All units in meters.
  auto exampleTrajectory = TrajectoryGenerator::GenerateTrajectory(
      // Start at the origin facing the +X direction
      Pose2d(0_m, 0_m, Rotation2d(0_deg)),
      // Pass through these two interior waypoints, making an 's' curve path
      {Translation2d(1_m, 1_m), Translation2d(2_m, -1_m)},
      // End 3 meters straight ahead of where we started, facing forward
      Pose2d(3_m, 0_m, Rotation2d(0_deg)),
      // Pass the config
      config);
}

void Autonomous::Loop()
{

}


void Autonomous::Stop()
{

}
