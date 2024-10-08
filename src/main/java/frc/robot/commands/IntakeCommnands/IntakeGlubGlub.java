// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommnands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.DELib.BooleanUtil.LatchedBolean;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeGlubGlub extends Command {
  /** Creates a new IntakeGlubGlub. */
  private IntakeSubsystem m_intake;
  private Timer m_timer;
  private LatchedBolean m_firstStage;
  private int i = 0;
  private boolean secondStage = false;
  private double intakePower = 0.3;
  private double slowOutake = 0.08;
  
  
  public IntakeGlubGlub(IntakeSubsystem intake) {
    m_intake = intake;
    m_timer = new Timer();
    m_firstStage = new LatchedBolean();
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_firstStage.reset();
    m_timer.reset();
    m_timer.start();
    i = 0;
    secondStage = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_firstStage.update(m_intake.hasGamePiece()); 
    if(i < 4 && m_firstStage.get()){
      if(m_intake.hasGamePiece() && m_timer.hasElapsed(0.1)){
        m_intake.setMotorPrecent(-intakePower);
        m_timer.reset();
        i++;
      }
      else if(!m_intake.hasGamePiece() && m_timer.hasElapsed(0.1)){
        m_intake.setMotorPrecent(intakePower);
        m_timer.reset();
      }
    }
    else if(i == 4 && m_intake.hasGamePiece()){
      m_intake.setMotorPrecent(-slowOutake);
    }
    else{
      m_intake.setMotorPrecent(0);
      secondStage = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // if(secondStage){
    //   m_intake.setPosition(8);
    // }
    // else{
    //   m_intake.setMotorPrecent(0);
    // }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return secondStage;
  }
}
