// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.DELib.Subsystems.PoseEstimator;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.DELib.Intepulation.InterpolatingDouble;
import frc.DELib.Intepulation.InterpolatingTreeMap;
import frc.DELib.Sensors.Pigeon;
import frc.DELib.Subsystems.Swerve.SwerveSubsystem;
import frc.DELib.Subsystems.Vision.VisionSubsystem;
import frc.DELib.Subsystems.Vision.VisionUtil.LimelightHelpers;

public class PoseEstimatorSubsystem extends SubsystemBase{
  /** Creates a new PoseEstimator. */
  private static SwerveSubsystem m_swerve;
  private static Pigeon m_gyro;
  private static SwerveDrivePoseEstimator m_poseEstimator;
  private static Pose2d m_estimatedRobotPose = new Pose2d();
  private static LimelightHelpers.PoseEstimate limelightMesermentMT2;
  private static InterpolatingTreeMap<InterpolatingDouble, Pose2d> m_pastPoses;
  private static double speakerHighetFromRobot = 2.049-0.136;
  private static double odometryToArmDistance = 0.13784;
  Field2d field2d;
  
  
  public PoseEstimatorSubsystem(SwerveSubsystem swerve) {
    field2d = new Field2d();
    m_swerve = swerve;
    m_gyro = Pigeon.getInstance();
    int k_maxPoseHistorySize = 51;
    m_pastPoses = new InterpolatingTreeMap<>(k_maxPoseHistorySize);
    m_poseEstimator = new SwerveDrivePoseEstimator(m_swerve.getKinematics(), Rotation2d.fromDegrees(0), m_swerve.getModulesPositions(), new Pose2d());
  }

  @Override
  public void periodic() {
    m_estimatedRobotPose = m_poseEstimator.update(m_gyro.getYaw(), m_swerve.getModulesPositions());
    updateVisionOdometry();
    m_pastPoses.put(new InterpolatingDouble(Timer.getFPGATimestamp()), getRobotPose());
    SmartDashboard.putString("estimatedRobotPose", getRobotPose().toString());
    SmartDashboard.putNumber("distance from speaker", getDistanceToBlueSpeaker());
    SmartDashboard.putNumber("angleSpeaker", getAngleToBlueSpeaker().getDegrees());
    field2d.setRobotPose(getRobotPose());
    SmartDashboard.putData("field2d ",field2d);
    SmartDashboard.putNumber("anglearmgoodhoo", getArmAngleToBlueSpeaker());
  }

  public static Pose2d getRobotPose(){
    return m_estimatedRobotPose;
  }

  private static void updateVisionOdometry(){
    boolean rejectUpdate = false;
    LimelightHelpers.SetRobotOrientation("limelight", m_swerve.getPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);
    limelightMesermentMT2 = VisionSubsystem.getEstimatedRobotPose();
    if(Math.abs(m_gyro.getRateStatusSignal().getValueAsDouble()) > 360 && getRobotPose().getX() < 5){
      rejectUpdate = true;
    }
    if(!rejectUpdate && VisionSubsystem.getTv()){
      m_poseEstimator.addVisionMeasurement(limelightMesermentMT2.pose, limelightMesermentMT2.timestampSeconds, VecBuilder.fill(0.7, 0.7, 9999999));
    }
  }

  public static void resetPosition(Pose2d pose){
    m_poseEstimator.resetPosition(m_gyro.getYaw(), m_swerve.getModulesPositions(), pose);
  }

  public static void resetPositionFromCamera(){
    resetPosition(limelightMesermentMT2.pose);
  }
  
  public static void zeroHeading(){
    Rotation2d heading = (DriverStation.getAlliance().isPresent() && (DriverStation.getAlliance().get() == DriverStation.Alliance.Red)) ? Rotation2d.fromDegrees(180) : new Rotation2d();
    m_poseEstimator.resetPosition(m_gyro.getYaw(), m_swerve.getModulesPositions(), new Pose2d(getRobotPose().getTranslation(), heading));
    m_gyro.setYaw(heading.getDegrees());
  }

  public static Pose2d getInterpolatedPose(double latencySeconds){
    double timestamp = Timer.getFPGATimestamp() - latencySeconds;
    return m_pastPoses.getInterpolated(new InterpolatingDouble(timestamp));
  }

  public static double getDistanceToBlueSpeaker(){
    return getRobotPose().getTranslation().getDistance(new Translation2d(0.0, 5.55));
  }

  public static Rotation2d getAngleToBlueSpeaker(){
    return Rotation2d.fromRadians(-Math.atan((5.55 - getRobotPose().getY())/(0 -getRobotPose().getX())));
  }

  public static double getArmAngleToBlueSpeaker(){
    double distanceToSpeaker = getDistanceToBlueSpeaker()+odometryToArmDistance + 0.06;
    return clamp(Math.toDegrees(Math.atan((speakerHighetFromRobot)/(distanceToSpeaker))), 10, 100);
  } 

   /**
  @param value clamped value
  @param min min value
  @param max max value
  @return sets a range for the value if its between the max and min points
  */
  private static double clamp(double value, double min, double max) {
    return Math.max(min, Math.min(max, value));
    }
}
