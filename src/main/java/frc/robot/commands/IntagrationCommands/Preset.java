// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntagrationCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class Preset extends InstantCommand {
 private ShooterSubsystem m_shooter;
 private ArmSubsystem m_arm;
 private double m_angle;
 private double m_velocity;
  /** Creates a new Preset. */
  public Preset(ShooterSubsystem shooter , ArmSubsystem arm, double angle ,double velocity) {
    m_shooter = shooter;
    m_arm = arm;
    m_angle = angle;
    m_velocity = velocity;
    addRequirements(arm , shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      m_arm.setMotionMagicPosition(m_angle);
      m_shooter.setMotionMagicVelocityWithRatio(m_velocity);
  }
}