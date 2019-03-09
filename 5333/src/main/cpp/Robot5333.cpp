#include "Robot5333.h"
#include "ControlMap.h"
#include "strategy/MPStrategy.h"

#include <math.h>
#include <iostream>

#include <cameraserver/CameraServer.h>

using namespace frc;
using namespace curtinfrc;

double lastTimestamp;

void Robot::RobotInit() {
  lastTimestamp = Timer::GetFPGATimestamp();

  // CameraServer::GetInstance()->StartAutomaticCapture(0);
  // CameraServer::GetInstance()->StartAutomaticCapture(1);

  // Our motors are mounted backwards, but the simulation doesn't know about that.
#ifdef __FRC_ROBORIO__
  robotmap.drivetrain.rightGearbox.transmission->SetInverted(true); 
#else
  robotmap.drivetrain.leftGearbox.transmission->SetInverted(true);
#endif
  robotmap.drivetrain.leftGearbox.encoder->ZeroEncoder();
  robotmap.drivetrain.rightGearbox.encoder->ZeroEncoder();

  drivetrain = new Drivetrain(robotmap.drivetrain.config, robotmap.drivetrain.gainsVelocity);
  drivetrain->SetDefault(std::make_shared<DrivetrainManualStrategy>(*drivetrain, robotmap.contGroup));
  drivetrain->GetConfig().gyro->Reset();
  drivetrain->StartLoop(100);
  stratFOC = std::make_shared<DrivetrainFOCStrategy>(*drivetrain, robotmap.contGroup, robotmap.drivetrain.gainsFOC);

  Register(drivetrain);
}

void Robot::RobotPeriodic() {
  double dt = Timer::GetFPGATimestamp() - lastTimestamp;
  lastTimestamp = Timer::GetFPGATimestamp();

  if (enableFOC && drivetrain->GetActiveStrategy() != stratFOC) enableFOC = false;
  if (robotmap.contGroup.GetButtonRise(ControlMap::activateFOC)) {
    enableFOC = !enableFOC;
    if (enableFOC) Schedule(stratFOC);
    else stratFOC->SetDone();
  }

  if (robotmap.contGroup.GetButtonRise(ControlMap::tapeAlign)) {
    double offset = drivetrain->GetConfig().gyro->GetAngle();
    offset += 0; // get angle from nt
    Schedule(std::make_shared<DrivetrainAngleStrategy>(*drivetrain, robotmap.drivetrain.gainsAlign, offset));
  }


  // Redundant, as it can already be accessed on shuffleboard via nt, but ~
  frc::SmartDashboard::PutNumber("Hatch Distance", robotmap.controlSystem.hatchDistanceEntry.GetDouble(-1));
  frc::SmartDashboard::PutNumber("Hatch X Offset", robotmap.controlSystem.hatchXoffsetEntry.GetDouble(0));
  frc::SmartDashboard::PutNumber("Hatch Y Offset", robotmap.controlSystem.hatchYoffsetEntry.GetDouble(0));
  
  frc::SmartDashboard::PutNumber("Tape Distance", robotmap.controlSystem.tapeDistanceEntry.GetDouble(-1));
  frc::SmartDashboard::PutNumber("Tape Angle", robotmap.controlSystem.tapeAngleEntry.GetDouble(0));
  frc::SmartDashboard::PutNumber("Tape Target", robotmap.controlSystem.tapeTargetEntry.GetDouble(-1));

  Update(dt);
}

void Robot::DisabledInit() {
  InterruptAll(true);
}

void Robot::AutonomousInit() {
  // Schedule(std::make_shared<PathfinderMPStrategy>(*drivetrain, robotmap.drivetrain.gainsPathfinder, "5333", "d2_bM"));
}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}
void Robot::TeleopPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}
