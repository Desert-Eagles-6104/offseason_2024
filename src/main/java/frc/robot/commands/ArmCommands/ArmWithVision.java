// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.DELib.Subsystems.Vision.VisionSubsystem;
import frc.robot.subsystems.ArmSubsystem;

public class ArmWithVision extends Command {
  /** Creates a new ArmWithVision. */
  ArmSubsystem m_arm;
  VisionSubsystem m_visionSubsystem;
  LinearFilter m_filterTy;

  private double Lastsetpoint = -9999;
  private double threshold = 0.05;


  public ArmWithVision(ArmSubsystem arm,VisionSubsystem visionSubsystem) {
    m_arm = arm;
    m_visionSubsystem = visionSubsystem;
    m_filterTy = LinearFilter.movingAverage(5);
    addRequirements(arm);
    }

  @Override
  public void initialize() {
    
  }


  @Override
  public void execute() {
    if(m_visionSubsystem.getTv()){
      if(Math.abs(Lastsetpoint - m_visionSubsystem.getTy()) > threshold){
      m_arm.setUsingInterpulation(m_filterTy.calculate(Lastsetpoint));
      Lastsetpoint = m_visionSubsystem.getTy();
      }
    }
  }


  @Override
  public void end(boolean interrupted) {

  }


  @Override
  public boolean isFinished() {
    return false;
  }
}
