// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.DELib.Subsystems.ServoSubsystem.ServoSubsystemConfiguration;
import frc.DELib.Subsystems.ServoSubsystem.Base.Motor.ServoSubsystemTalon;

public class ArmSubsystem extends ServoSubsystemTalon {
  public ArmSubsystem(ServoSubsystemConfiguration configuration) {
    super(configuration);
  }

  @Override
  public void periodic() {
    super.periodic();
  }
}
