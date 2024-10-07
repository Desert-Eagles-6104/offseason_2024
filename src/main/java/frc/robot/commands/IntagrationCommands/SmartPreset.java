// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntagrationCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class SmartPreset extends Command {
  private ShooterSubsystem m_shooter;
 private ArmSubsystem m_arm;
 private double m_angle = 0;
 private double m_velocity = 0;
 private boolean m_spin = true;

  /** Creates a new Preset. */
  public SmartPreset(ShooterSubsystem shooter , ArmSubsystem arm, double angle ,double velocity) {
    m_shooter = shooter;
    m_arm = arm;
    m_angle = angle;
    m_velocity = velocity;
    m_spin = true;
    addRequirements(arm , shooter);
  }

  public SmartPreset(ShooterSubsystem shooter , ArmSubsystem arm, double angle ,double velocity, boolean spin) {
    m_shooter = shooter;
    m_arm = arm;
    m_angle = angle;
    m_velocity = velocity;
    m_spin = spin;
    addRequirements(arm , shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.setMotionMagicPosition(m_angle);
    if(m_spin){
      m_shooter.setMotionMagicVelocityWithRatio(m_velocity);
    }
    else{
      m_shooter.setMotionMagicVelocity(m_velocity);
    }
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_arm.isAtSetpoint() && m_shooter.isAtSetpoint();
  }
}
