// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.DELib.Intepulation.LinearInterpolator;
import frc.DELib.Sensors.BeamBreak;
import frc.DELib.Subsystems.ServoSubsystem.ServoSubsystemConfiguration;
import frc.DELib.Subsystems.ServoSubsystem.Base.Motor.ServoSubsystemTalon;

public class ArmSubsystem extends ServoSubsystemTalon {
  private BeamBreak m_armMagnet;
  private boolean magnetState = false;
  private LinearInterpolator linearVision;
  private LinearInterpolator linearInterpolatorLocalizationSpeaker;
  private LinearInterpolator linearInterpulationDeliverySpeaker;
  
  
  public ArmSubsystem(ServoSubsystemConfiguration configuration) {
    super(configuration);
    m_armMagnet = new BeamBreak(0);
    linearVision = new LinearInterpolator(interpulationVisionSpeaker);
    linearInterpolatorLocalizationSpeaker = new LinearInterpolator(interpulationLocalizationSpeaker);
    linearInterpulationDeliverySpeaker = new LinearInterpolator(interpulationDeliverySpeaker);
    SmartDashboard.putNumber("ArmAngleToSet", 90);
    SmartDashboard.putNumber("ArmAngleOffset", 0.75);
  }

  @Override
  public void setMotionMagicPosition(double position) {
    super.setMotionMagicPosition(position);
  }

  public void setPosition(double position){
    if(Math.abs(super.getClosedLoopError()) < 2){
      super.setPosition(position);
      SmartDashboard.putNumber("slot", 1);
    }
    else{
      super.setMotionMagicPosition(position);
      SmartDashboard.putNumber("slot", 0);
    }
  }

  @Override
  public void periodic() {
    super.periodic();
    magnetState = magnetUpdate();
    SmartDashboard.putBoolean("magnetcontact", magnetState);
    SmartDashboard.putNumber("armSetpoint", setpoint);
  }

  public boolean magnetUpdate(){
    m_armMagnet.update();
    return m_armMagnet.get();
  }

  public boolean getMagnetState(){
    return magnetState;
  }

  public double getSetpoint(){
    return setpoint;
  }

  public double getInterpulatedAngleVision(double limeVal){
    return linearVision.getInterpolatedValue(limeVal);
  }

  public void setUsingInterpulationVision(double value) {
    double angle = linearVision.getInterpolatedValue(value);
    this.setPosition(angle);
  }

  public double getInterpulatedAnglePOS(double limeVal){
    return linearInterpolatorLocalizationSpeaker.getInterpolatedValue(limeVal);
  }

  public void setUsingInterpulationPOS(double value) {
    double angle = linearInterpolatorLocalizationSpeaker.getInterpolatedValue(value);
    this.setPosition(angle+SmartDashboard.getNumber("ArmAngleOffset", 0.75));
  }

  public double getInterpulationDeliverySpeaker(double limeVal){
    return linearInterpulationDeliverySpeaker.getInterpolatedValue(limeVal);
  }

  public void setInterpulationDeliverySpeaker(double value) {
    double angle = linearInterpulationDeliverySpeaker.getInterpolatedValue(value);
    this.setPosition(angle);
  }

  public void Print(){
    setPosition(SmartDashboard.getNumber("ArmAngleToSet", 90));
  }
  //position Interpulation
  double[][] interpulationLocalizationSpeaker = 
  {{1.32, 55.0},
  {1.57, 51.0},
  {1.88, 47.0},
  {2.20, 43.0},
  {2.30, 42.0},
  {2.47, 41.0},
  {2.77, 38},
  {3.1, 35.5},
  {3.38, 33.5},
  {3.74, 33.0},
  {4.12, 31.0},
  {4.31, 29.75},
  {4.62, 28.8},
  {4.94,27.7},
  {5.21, 27.25},
  {5.51, 26.35},
  {5.80, 26.15},
  {6.104, 25.85},
  {6.47,25.75},
  {6.76, 25.75},
  {7.06, 25.20},
  {7.41, 24.5}};

 


  //vision Intepulation
  double[][] interpulationVisionSpeaker = 
  {
    {27.19,47},
    {16.72, 39},
    {13.65, 36.5},
    {10.91, 34},
    {9.06, 33},
    {7.48, 31},
    {5.95, 30.25},
    {4.68, 29.1},
    {3.63, 28.25},
    {3.04, 27.7},
    {2.30, 27.25},
    {1.89, 26},
    {1.64, 26.75},
    {1.54,26.5},
    {0.90, 26.4},
    {0.57, 26},
    {0.12, 25.8},
    {-0.72, 25.4},
    {-0.53, 25.4},
    {-1.51, 24.9}};

  double[][] interpulationDeliverySpeaker = 
  {
    {1.183, 57.0},
    {1.48, 51.0},
    {1.779, 47.0},
    {2.60, 39.0},
    {2.92, 36.5},
    {3.29, 34.0},
    {3.51, 33.0},
    {3.95, 31.0},
    {4.15, 30.25},};
}