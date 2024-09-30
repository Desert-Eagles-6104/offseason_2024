// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.DELib.Subsystems.Swerve.SwerveCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.DELib.Subsystems.Swerve.SwerveSubsystem;
import frc.DELib.Subsystems.Swerve.SwerveUtil.HeadingController;
import frc.DELib.Subsystems.Vision.VisionSubsystem;

public class RotateToTarget extends Command {
  SwerveSubsystem m_swerve;
  VisionSubsystem m_VisionSubsystem;
  HeadingController m_headingController;
  CommandPS5Controller m_controller;
  /** Creates a new RotateToTarget. */
  public RotateToTarget(SwerveSubsystem swerve , VisionSubsystem vision, CommandPS5Controller controller) {
    m_swerve = swerve;
    m_VisionSubsystem = vision;
    m_controller = controller;
    m_headingController = new HeadingController(0.1, 0, 0);
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double errorDegrees = m_VisionSubsystem.getTv() ? -m_VisionSubsystem.getTx() : 0;
    Rotation2d target = m_swerve.getInterpolatedPose(m_VisionSubsystem.getTotalLatency()).getRotation().plus(Rotation2d.fromDegrees(errorDegrees));
    m_headingController.setSetpoint(target);
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(m_controller.getLeftX(), m_controller.getLeftY(), m_headingController.update(m_swerve.getInterpolatedPose(m_VisionSubsystem.getTotalLatency()).getRotation()));
    m_swerve.drive(chassisSpeeds, true, true, new Translation2d());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
