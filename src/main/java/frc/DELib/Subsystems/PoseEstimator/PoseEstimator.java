// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.DELib.Subsystems.PoseEstimator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.DELib.Intepulation.InterpolatingDouble;
import frc.DELib.Intepulation.InterpolatingTreeMap;
import frc.DELib.Subsystems.PoseEstimator.PoseEstimatorUtil.PoseMergingFOM;
import frc.DELib.Subsystems.Swerve.SwerveSubsystem;
import frc.DELib.Subsystems.Vision.VisionSubsystem;

public class PoseEstimator extends SubsystemBase{
  /** Creates a new PoseEstimator. */
  private PoseMergingFOM m_poseMerging;
  private SwerveSubsystem m_swerve;
  private VisionSubsystem m_vision;
  private InterpolatingTreeMap<InterpolatingDouble, Pose2d> m_pastPoses;
  private Pose2d m_visionPose = new Pose2d();
  private Pose2d m_estimatedRobotPose = new Pose2d();

  public PoseEstimator(SwerveSubsystem swerve, VisionSubsystem vision) {
    m_poseMerging = new PoseMergingFOM();
    m_swerve = swerve;;
    m_vision = vision;
    int k_maxPoseHistorySize = 51;
    m_pastPoses = new InterpolatingTreeMap<>(k_maxPoseHistorySize);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(m_vision.getTv()) m_poseMerging.updateCameraFOM(0.1); else m_poseMerging.updateCameraFOM(Double.POSITIVE_INFINITY);
    if(m_vision.getTv()) m_visionPose = m_vision.getEstimatedRobotPose();
    double latencySeconds = m_vision.getTotalLatency(); 
    m_estimatedRobotPose = m_poseMerging.FOMCalculation(m_swerve.getInterpolatedPose(latencySeconds), m_visionPose);
    SmartDashboard.putString("Estimated Pose FOM", m_estimatedRobotPose.toString());
    // m_swerve.resetOdometry(m_estimatedRobotPose); //TODO: cheak on robot
  }

  public Pose2d getRobotPose(){
    return m_estimatedRobotPose;
  }

  public Pose2d getInterpolatedPose(double latencySeconds){
    double timestamp = Timer.getFPGATimestamp() - latencySeconds;
    return m_pastPoses.getInterpolated(new InterpolatingDouble(timestamp));
  }
}
