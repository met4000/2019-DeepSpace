#pragma once

#include <frc/Timer.h>
#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <frc/SpeedControllerGroup.h>
#include <frc/DoubleSolenoid.h>
#include "CurtinControllers.h"

#include "CurtinCtre.h"
#include "Gearbox.h"
#include "Drivetrain.h"

#include "strategies/DriveStrategies.h"


class Robot : public frc::TimedRobot, protected curtinfrc::StrategyController {
 public:
  void RobotInit() override;

  void AutonomousInit() override;
  void AutonomousPeriodic() override;

  void TeleopInit() override;
  void TeleopPeriodic() override;

  void TestInit() override;
  void TestPeriodic() override;

  curtinfrc::XboxController xbox{0};
  curtinfrc::ControllerGroup contGroup{xbox};

  curtinfrc::Talon *leftMotors[1], *rightMotors[1];
  curtinfrc::Gearbox *left, *right;
  curtinfrc::Drivetrain *drivetrain;
};
