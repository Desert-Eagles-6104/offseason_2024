// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommnands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.DELib.BooleanUtil.LatchedBolean;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeGlubGlub extends Command {
  /** Creates a new IntakeGlubGlub. */
  private IntakeSubsystem m_intake;
  private Timer m_timer;
  private LatchedBolean m_firstStage;
  private boolean m_isFirstBeamBreak;
  private int i = 0;
  private boolean secondStage = false;
  private double intakePower = 0.3;
  private double slowOutake = 0.1;
  private int m_howManyGlubGlub = 4;
  private boolean Ligma = false;
  
  
  public IntakeGlubGlub(IntakeSubsystem intake, Boolean isFirstBeambreak) {
    m_intake = intake;
    m_timer = new Timer();
    m_firstStage = new LatchedBolean();
    m_isFirstBeamBreak = isFirstBeambreak;
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
    Ligma = false;
    if(!m_isFirstBeamBreak){
      m_howManyGlubGlub = 5;
    }
    else{
      m_howManyGlubGlub = 2;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_isFirstBeamBreak){
      SmartDashboard.putString("isIntakeFirst", "yes");
      m_firstStage.update(m_intake.firstBeamBreak()); 
      if(i < m_howManyGlubGlub && m_firstStage.get()){
        if(m_intake.firstBeamBreak() && m_timer.hasElapsed(0.1)){
          m_intake.setMotorPrecent(-intakePower);
          m_timer.reset();
          i++;
        }
        else if(!m_intake.firstBeamBreak() && m_timer.hasElapsed(0.1)){
          m_intake.setMotorPrecent(intakePower);
          m_timer.reset();
        }
      }
      else{
        m_intake.setMotorPrecent(0);
        secondStage = true;
      }
    }
    else{
      SmartDashboard.putString("isIntakeFirst", "NO");
      SmartDashboard.putBoolean("t", Ligma);
      m_firstStage.update(m_intake.secondBeamBreak()); 
      if(i < m_howManyGlubGlub && m_firstStage.get()){
        if(m_intake.secondBeamBreak() && m_timer.hasElapsed(0.05)){
          m_intake.setMotorPrecent(-intakePower);
          m_timer.reset();
          i++;
        }
        else if(!m_intake.secondBeamBreak() && m_timer.hasElapsed(0.05)){
          m_intake.setMotorPrecent(intakePower);
          m_timer.reset();
        }
      }
      else if(i == m_howManyGlubGlub && m_intake.secondBeamBreak() && !Ligma){
        Ligma = true;
        SmartDashboard.putString("stateIntake", "seesNote");
        m_intake.setMotorPrecent(-slowOutake);
      }
      else if(i == m_howManyGlubGlub && !m_intake.secondBeamBreak()){
        if(Ligma){
          secondStage = !m_intake.secondBeamBreak();
          SmartDashboard.putString("stateIntake", "seesNote");
        }
        else{
          SmartDashboard.putString("stateIntake", "NOTseesNote");
          m_intake.setMotorPrecent(slowOutake);
          // secondStage = m_intake.secondBeamBreak();
        }
      }
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
    SmartDashboard.putBoolean("t", Ligma);
    Ligma = false;
    m_intake.setMotorPrecent(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return secondStage;
  }
}
