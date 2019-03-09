#include "strategies/DriveStrategies.h"

#include <cmath>

// #include "ControlMap.h"

using namespace curtinfrc;

BaseDrivetrainTeleopStrategy::BaseDrivetrainTeleopStrategy(std::string name, Drivetrain &drivetrain, ControllerGroup &contGroup)
    : Strategy(name), _drivetrain(drivetrain), _contGroup(contGroup) {
  Requires(&drivetrain);
  SetCanBeInterrupted(true);
  SetCanBeReused(true);
} 

DrivetrainManualStrategy::DrivetrainManualStrategy(Drivetrain &drivetrain, ControllerGroup &contGroup) : BaseDrivetrainTeleopStrategy("Drivetrain Manual", drivetrain, contGroup) { }

void DrivetrainManualStrategy::OnUpdate(double dt) {
  double joyForward = 0, joyTurn = 0;
  
  joyForward = -_contGroup.GetCircularisedAxisAgainst({ 0, 1 }, { 0, 0 }) * 0.9;
  joyForward *= std::abs(joyForward);

  joyTurn = _contGroup.GetCircularisedAxisAgainst({ 0, 0 }, { 0, 1 }) * 0.7;
  // joyTurn *= abs(joyTurn);

  double leftSpeed = joyForward + joyTurn;
  double rightSpeed = joyForward - joyTurn;

  if (_invertedToggle.Update(_contGroup.GetButton({ 0, 2 })))
    _drivetrain.SetInverted(!_drivetrain.GetInverted());

  _drivetrain.Set(leftSpeed, rightSpeed);
}

DrivetrainAngleStrategy::DrivetrainAngleStrategy(Drivetrain &drivetrain, control::PIDGains gains, double angle) : Strategy("Drivetrain Angle"), _drivetrain(drivetrain), _pid(gains) {
  Requires(&drivetrain);
  SetCanBeInterrupted(true);
  SetCanBeReused(false);

  _angle = fmod(angle + (_drivetrain.GetInverted() ? 180 : 360), 360);
  _pid.SetSetpoint(_angle);
  _pid.SetWrap(360.0);

  _drivetrain.SetExternalLoop([&](Drivetrain &, double dt) {
    double offset = _pid.Calculate(_drivetrain.GetConfig().gyro->GetAngle(), dt);
    return std::pair<double, double>(offset, -offset);
  });
}

void DrivetrainAngleStrategy::OnUpdate(double dt) {
  if (_pid.IsDone())
    SetDone();
}
