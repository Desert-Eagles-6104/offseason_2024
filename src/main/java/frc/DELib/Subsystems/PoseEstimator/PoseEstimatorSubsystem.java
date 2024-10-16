// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.DELib.Subsystems.PoseEstimator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.DELib.BooleanUtil.StableBoolean;
import frc.DELib.Sensors.Pigeon;
import frc.DELib.Subsystems.Swerve.SwerveSubsystem;
import frc.DELib.Subsystems.Vision.VisionSubsystem;
import frc.DELib.Subsystems.Vision.VisionUtil.LimelightHelpers;

public class PoseEstimatorSubsystem extends SubsystemBase{
  /** Creates a new PoseEstimator. */
  private static SwerveSubsystem m_swerve;
  private static Pigeon m_gyro;
  private static LimelightHelpers.PoseEstimate limelightMesermentMT2;
  private static double speakerHighetFromRobot = 2.049-0.136;
  private static double odometryToArmDistance = 0.13784;
  private static boolean first = true;
  private static StableBoolean tvStableBoolean;
  Field2d field2d;
  
  
  public PoseEstimatorSubsystem(SwerveSubsystem swerve) {
    field2d = new Field2d();
    m_swerve = swerve;
    m_gyro = Pigeon.getInstance();
    tvStableBoolean = new StableBoolean(0.15);
  }

  @Override
  public void periodic() {
    if(!first){
      updateVisionOdometry();
      // SmartDashboard.putString("estimatedRobotPose", getRobotPose().toString());
      SmartDashboard.putNumber("distance from speaker", getDistanceToBlueSpeaker());
      SmartDashboard.putNumber("angleSpeaker", getAngleToBlueSpeaker().getDegrees());
      // field2d.setRobotPose(getRobotPose());
      // SmartDashboard.putData("field2d ",field2d);
      SmartDashboard.putBoolean("isCentered", isCentered());
    }
    else{
      first = false;
    }
  }

  private static void updateVisionOdometry(){
    if(!first){
      boolean rejectUpdate = false;
      LimelightHelpers.SetRobotOrientation("limelight", getRobotPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);
      limelightMesermentMT2 = VisionSubsystem.getEstimatedRobotPose();
      if(Math.abs(m_gyro.getRateStatusSignal().getValueAsDouble()) > 360 && getRobotPose().getX() < 5 || limelightMesermentMT2.pose == null){
        rejectUpdate = true;
      }
      if(!rejectUpdate && tvStableBoolean.get(VisionSubsystem.getTv()) && limelightMesermentMT2.pose != null){
        m_swerve.addVisionMeasurement(limelightMesermentMT2.pose, limelightMesermentMT2.timestampSeconds);
      }
    }
    else{
      first = false;
    }
  }

  public static boolean isCentered(){
    return Math.abs(getHeading().getDegrees() - getAngleToBlueSpeaker().getDegrees()) < 1.5; //TODO: cheak
  }

  public static Pose2d getRobotPose(){
    return m_swerve.getPose();
  }

  public static void resetPosition(Pose2d pose){
    m_swerve.resetOdometry(pose);
  }

  public static void resetPositionFromCamera(){
    if(limelightMesermentMT2.pose != null){
      resetPosition(limelightMesermentMT2.pose);
    }
  }
  
  public static void zeroHeading(){
    m_swerve.zeroHeading();
  }

    public static Rotation2d getHeading() {
    return getRobotPose().getRotation();
  }

  public static Pose2d getInterpolatedPose(double latencySeconds){
    return m_swerve.getInterpolatedPose(latencySeconds);
  }

    public static Rotation2d getAngleToDeliveryCloseToSpeaker(){
    return Rotation2d.fromRadians(-Math.atan((7.0 - getRobotPose().getY())/(0.95 -getRobotPose().getX())));
  }

    public static double getDistanceToDeliveryCloseToSpeaker(){
    return getRobotPose().getTranslation().getDistance(new Translation2d(0.95, 7.0));
  }

  public static double getDistanceToBlueSpeaker(){
    return getRobotPose().getTranslation().getDistance(new Translation2d(0.0, 5.55));
  }

  public static Rotation2d getAngleToBlueSpeaker(){
    return Rotation2d.fromRadians(-Math.atan((5.55 - getRobotPose().getY())/(0 -getRobotPose().getX())));
  }

  public static double getArmAngleToBlueSpeaker(){
    double distanceToSpeaker = getDistanceToBlueSpeaker()+odometryToArmDistance;
    return clamp(Math.toDegrees(Math.atan((speakerHighetFromRobot)/(distanceToSpeaker)))+7.9  , 10, 100);
  } 

  
  public static double getDistanceToBlueSpeakerOnTheMove(){
    return getInterpolatedPose(0.02).getTranslation().getDistance(new Translation2d(0.0, 5.55));
  }

  public static Rotation2d getAngleToBlueSpeakerOnTheMove(){
    Pose2d pose = getInterpolatedPose(0.02);
    return Rotation2d.fromRadians(-Math.atan((5.55 - pose.getY())/(0 -pose.getX())));
  }

  public static double getArmAngleToBlueSpeakerOnTheMove(){
    double distanceToSpeaker = getDistanceToBlueSpeakerOnTheMove()+odometryToArmDistance - 0.1;
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
