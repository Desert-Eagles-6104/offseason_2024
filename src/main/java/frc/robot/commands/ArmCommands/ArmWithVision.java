// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.DELib.Subsystems.Vision.VisionSubsystem;
import frc.robot.Constants.Vision;
import frc.robot.subsystems.ArmSubsystem;

public class ArmWithVision extends Command {
  /** Creates a new ArmWithVision. */
  ArmSubsystem m_armSubsystem;
  VisionSubsystem m_visionSubsystem;
  MedianFilter m_filter;


  public ArmWithVision(ArmSubsystem armSubsystem,VisionSubsystem visionSubsystem) {
    m_armSubsystem = armSubsystem;
    m_visionSubsystem = visionSubsystem;
    m_filter = new MedianFilter(5);
    }

  @Override
  public void initialize() {
    
  }


  @Override
  public void execute() {
    if(m_visionSubsystem.getTv()){
      m_armSubsystem.setMotionMagicPosition(m_armSubsystem.getInterpulatedAngle(m_filter.calculate(m_visionSubsystem.getTy())));
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
