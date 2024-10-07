// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterSetVelocitySmart extends Command {
  /** Creates a new ShooterSetVelocity. */
  private ShooterSubsystem m_shooter;
  private double m_velocity = 0;
  public ShooterSetVelocitySmart(ShooterSubsystem shooter, double velocity) {
    m_shooter = shooter;
    m_velocity = velocity;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    m_shooter.setMotionMagicVelocityWithRatio(m_velocity);
  }

  @Override
  public void execute() {
      
  }

  @Override
  public void end(boolean interrupted) {
      
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
