// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.DELib.Subsystems.Swerve.SwerveUtil;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.DELib.Subsystems.Swerve.SwerveSubsystem;
import frc.DELib.Subsystems.Vision.VisionSubsystem;

public class DriveAssistAuto extends Command {
  private SwerveSubsystem m_swerveSubsystem;
  private double m_kpSide = 0.03;
  private double m_kpForward = 0.08;
  private LinearFilter m_filterSide;
  private LinearFilter m_filterForward;
  private ChassisSpeeds chassisSpeeds;
  private Translation2d centerOfRobot = new Translation2d();

  public DriveAssistAuto(SwerveSubsystem swerveSubsystem){
    m_swerveSubsystem = swerveSubsystem;
    m_filterSide = LinearFilter.movingAverage(4);
    m_filterForward = LinearFilter.movingAverage(4);
    chassisSpeeds = new ChassisSpeeds();
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    if(VisionSubsystem.getTvNote()){
      chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(m_filterForward.calculate(-VisionSubsystem.getTyNote())*m_kpForward, m_filterSide.calculate(-VisionSubsystem.getTxNote())*m_kpSide, 0, m_swerveSubsystem.getHeading());
    }
    m_swerveSubsystem.drive(chassisSpeeds, false, true, centerOfRobot);
  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
