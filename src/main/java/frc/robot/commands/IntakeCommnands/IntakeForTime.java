// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommnands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeForTime extends Command {
  /** Creates a new IntakeForTime. */
  private IntakeSubsystem m_intake;
  private double m_power;
  private double m_timeToFinish;
  private Timer m_timer;
  public IntakeForTime(IntakeSubsystem intake, double power, double timeToFinish) {
    m_intake = intake;
    m_power = power;
    m_timeToFinish = timeToFinish;
    m_timer = new Timer();
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
    m_intake.setMotorPrecent(m_power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.setMotorPrecent(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(m_timeToFinish);
  }
}
