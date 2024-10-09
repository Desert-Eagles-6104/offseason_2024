// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommnands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeEatUntilHasNote extends Command {

  IntakeSubsystem m_IntakeSubsystem;
  double m_precent;
  boolean m_isFirst = true;

  public IntakeEatUntilHasNote(IntakeSubsystem intakeSubsystem, double precent, boolean isFirst) {
    m_IntakeSubsystem = intakeSubsystem;
    m_precent = precent;
    m_isFirst = isFirst;
    addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() {
    m_IntakeSubsystem.setMotorPrecent(m_precent);
  }

  @Override
  public void execute() {

  }

  @Override
  public void end(boolean interrupted) {
    m_IntakeSubsystem.disableMotors();
  }

  @Override
  public boolean isFinished() {
    if(m_isFirst){
      return m_IntakeSubsystem.firstBeamBreak();
    }
    else{
      return m_IntakeSubsystem.secondBeamBreak();
    }
  }
}
