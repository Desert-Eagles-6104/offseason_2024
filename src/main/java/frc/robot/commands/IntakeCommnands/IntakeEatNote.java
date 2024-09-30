// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommnands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.DELib.BooleanUtil.LatchedBolean;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeEatNote extends Command {
  private IntakeSubsystem m_intake;
  private final double EAT_NOTE_PRECENTAGE = -0.8;
  private final double FIRST_STAGE_PRECENT = 0.5;
  private final double SECOAND_STAGE_MOVE_POSITION = 0.2;
  private LatchedBolean m_firstStage;
  private boolean m_secondStage;
  private boolean m_thirdStage;
  private Timer m_timer;

  public IntakeEatNote(IntakeSubsystem intake) {
    m_intake = intake;
    m_timer = new Timer();
    m_firstStage = new LatchedBolean();
    addRequirements(intake);
  }

  
  @Override
  public void initialize() {
    m_firstStage.reset();
    m_secondStage = false;
    m_thirdStage = false;
    m_intake.setMotorPrecent(EAT_NOTE_PRECENTAGE);
  }

  
  @Override
  public void execute() {
    m_firstStage.update(m_intake.hasGamePiece());
    firstStage();
    secondStage();
  }

  
  @Override
  public void end(boolean interrupted) {
    m_intake.disableMotors();
  }

  
  @Override
  public boolean isFinished() {
    return m_thirdStage;
  }

  private void firstStage(){
    double sign = -1;
    int i = 0;
    if(m_firstStage.get() && !m_secondStage && !m_thirdStage){
      m_timer.restart();
      m_intake.setMotorPrecent(FIRST_STAGE_PRECENT * sign);
    }
    while (m_firstStage.get() && !m_secondStage && !m_thirdStage) {
      if(m_timer.hasElapsed(0.5) && (sign < 0)){
        sign = sign * sign;
        m_intake.setMotorPrecent(FIRST_STAGE_PRECENT * sign);
        m_timer.restart();
        i++;
        if(i == 3){
          m_secondStage = true;
          break;
        }
      }
    }
  }

  public void secondStage(){
    if(m_secondStage && !m_thirdStage){
      m_intake.setPosition(SECOAND_STAGE_MOVE_POSITION);
      m_thirdStage = true;
    }
  }
}
