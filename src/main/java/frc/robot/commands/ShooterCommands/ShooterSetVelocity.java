// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterSetVelocity extends InstantCommand {
  /** Creates a new ShooterSetVelocity. */
  private ShooterSubsystem m_shooter;
  private double m_velocity = 0;
  public ShooterSetVelocity(ShooterSubsystem shooter, double velocity) {
    m_shooter = shooter;
    m_velocity = velocity;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    m_shooter.setMotionMagicVelocityWithRatio(m_velocity);
  }
}
