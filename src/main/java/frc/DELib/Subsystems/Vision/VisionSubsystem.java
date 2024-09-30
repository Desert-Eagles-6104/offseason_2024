// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.DELib.Subsystems.Vision;

import java.io.IOException;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.DELib.Subsystems.Vision.VisionUtil.CameraSettings;
import frc.DELib.Subsystems.Vision.VisionUtil.LimelightHelpers;

public class VisionSubsystem extends SubsystemBase {
  /** Creates a new VisionSubsystem. */
  
  private CameraSettings m_aprilTagCameraSettings = null;
  private double m_tx = 0; //Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees)
  private double m_ty = 0; //Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees)
  private boolean m_tv = false; //Whether the limelight has any valid targets (0 or 1)
  private Pose2d m_estimatedRobotPose = new Pose2d(); 
  private double m_currentID = 0;
  
  private double cropXMin = -1;
  private double cropXMax = 1;
  private double cropYMin = -1;
  private double cropYMax = 1;
  
  double xFOV = 62.5;
  double yFOV = 48.9;
  
  int regularPipeline = 0;
  int pipelineX2 = 1;

  private CameraSettings m_gamePieceCameraSettings = null;
  private double m_gamePieceTX;
  private double m_gamePieceTY;

  private AprilTagFieldLayout aprilTagFieldLayout = null;

  private Field2d field2d = new Field2d();

  //*create a new VisionSubsystem constructor to apply the subsystem's properties */
  public VisionSubsystem(CameraSettings aprilTagCameraSettings, CameraSettings gamePieceCameraSettings) {
    try {
      aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.name());
    } catch (IOException e) {
      System.out.println("april tag field layout not found!");
    }
    m_aprilTagCameraSettings = aprilTagCameraSettings;
    if(aprilTagCameraSettings != null){
      LimelightHelpers.setCameraPose_RobotSpace(CameraType.AprilTagCamera.getCameraName(), aprilTagCameraSettings.m_forward, aprilTagCameraSettings.m_Side, aprilTagCameraSettings.m_up, aprilTagCameraSettings.m_roll, aprilTagCameraSettings.m_pitch, aprilTagCameraSettings.m_yaw);
    }
    
    // m_gamePieceCameraSettings = gamePieceCameraSettings;
    // if(gamePieceCameraSettings != null){
    //   LimelightHelpers.setCameraPose_RobotSpace(CameraType.GamePieceCamera.getCameraName(), gamePieceCameraSettings.m_forward, gamePieceCameraSettings.m_Side, gamePieceCameraSettings.m_up, gamePieceCameraSettings.m_roll, gamePieceCameraSettings.m_pitch, gamePieceCameraSettings.m_yaw);
    // }
  }

  @Override
  public void periodic() {
    m_tv = LimelightHelpers.getTV(CameraType.AprilTagCamera.getCameraName());
    if(m_tv){
      m_tx = LimelightHelpers.getTX(CameraType.AprilTagCamera.getCameraName());
      m_ty = LimelightHelpers.getTY(CameraType.AprilTagCamera.getCameraName());
      m_currentID = LimelightHelpers.getFiducialID(CameraType.AprilTagCamera.getCameraName());
      m_estimatedRobotPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(CameraType.AprilTagCamera.getCameraName()).pose;
    }

    //bounding april tag
    // orbitCalculation();
    SmartDashboard.putData("Field ", field2d);
    field2d.setRobotPose(m_estimatedRobotPose);
    //limelight values
    SmartDashboard.putNumber("TX", getTx());
    SmartDashboard.putNumber("TY", getTy());
    SmartDashboard.putBoolean("TV", getTv());

    SmartDashboard.putString("3D", m_estimatedRobotPose.toString());
    SmartDashboard.putNumber("Dis", getDistance());
    // SmartDashboard.putNumber("DistanceFromTargetX", getDstX(Constants.Vision.tragetHeight));
    // SmartDashboard.putNumber("DistanceFromTargetY", getDstY(Constants.Vision.tragetHeight));

    // if(m_gamePieceCameraSettings != null){
    //   m_gamePieceTX = LimelightHelpers.getTX(CameraType.GamePieceCamera.getCameraName());
    //   m_gamePieceTY = LimelightHelpers.getTY(CameraType.GamePieceCamera.getCameraName());
    // }
  }

  public double getDistance(){
    if(getTv() ){
      double highet = 1.368552 + 0.08255 - 0.10689;
      return highet / Math.tan(getTy());
    }
    return 0;
  }

  public Pose2d getEstimatedRobotPose(){
    return m_estimatedRobotPose;
  }
  
  public double getTx(){
    return m_tx;
  }

  public double getGamePieceTx(){
    return m_gamePieceTX;
  }
  
  public double getTy(){
    return m_ty + m_aprilTagCameraSettings.m_pitch;
  }

  public double getGamePieceTy(){
    return m_gamePieceTY;
  }

  public boolean getTv(){
    return m_tv;
  }

  public void crop(double cropXMin, double cropXMax, double cropYMin, double cropYMax){
    LimelightHelpers.setCropWindow(CameraType.AprilTagCamera.getCameraName(), cropXMin, cropXMax, cropYMin, cropYMax);
  }
  
  /**   
   * @return Total vision latency (photons -> robot) in seconds
   */
  public double getTotalLatency() {
    double miliToSec = 0.001;
    return LimelightHelpers.getLatency_Pipeline(CameraType.AprilTagCamera.getCameraName()) + LimelightHelpers.getLatency_Capture(CameraType.AprilTagCamera.getCameraName()) * miliToSec;
  }

  public int getCurrentID(){
    return (int)m_currentID;
  }

  /*
   * here we are caculate our crop setting we are doing in by using the camera fov and the limelight values to crop the pic
   * we are doing this because crop the full picture to maxmize the limelight FPS
   */
  public void orbitCalculation(){
    double precent = 0.5;
    double outerLayer = 15.0;
    double _xFOV = xFOV;
    double _yFOV = yFOV;

    if(LimelightHelpers.getCurrentPipelineIndex(CameraType.AprilTagCamera.getCameraName()) == pipelineX2){
      _xFOV = xFOV / 2.0;
      _yFOV = yFOV / 2.0;
      outerLayer = outerLayer / 2;
    }

    if(m_tv){
      cropXMin = (getTx() - outerLayer) / (precent * _xFOV);
      cropXMax = (getTx() + outerLayer) / (precent * _xFOV);
      cropYMin = (getTy() - outerLayer) / (precent * _yFOV);
      cropYMax = (getTy() + outerLayer) / (precent * _yFOV);
    }
    else{
      cropXMin = -1.0;
      cropXMax = 1.0;
      cropYMin = -1.0;
      cropYMax = 1.0;
    }
    crop( cropXMin , cropXMax , cropYMin , cropYMax );
  }

  // public double getDstX(double targetHeight){
  //   double ty = getGamePieceTx();
  //   return  (targetHeight-Constants.Vision.cameraHeight)/Math.tan(Math.toRadians(ty));
  // }

  // public double getDstY( double targetHeight){
  //   double tx = getGamePieceTx();
  //   double ty = getGamePieceTy();
  //   double distanceX = (targetHeight-Constants.Vision.cameraHeight)/Math.tan(Math.toRadians(ty));
  //   return distanceX/Math.tan(Math.toRadians(tx));
  // }
 
  //  public Translation2d getDistance(double targetHeight){
  //   return new Translation2d(getDstX(targetHeight),getDstY(targetHeight));
  // }

  public void changePiplne(int pipeline){
    LimelightHelpers.setPipelineIndex(CameraType.AprilTagCamera.getCameraName(), pipeline);
  }

  public enum CameraType{
    AprilTagCamera("limelight"),
    GamePieceCamera("GamePieceCamera");

    final String m_cameraName;

    CameraType(String cameraName){
      m_cameraName = cameraName;
    }

    public String getCameraName() {
        return m_cameraName;
    }
  }
}

