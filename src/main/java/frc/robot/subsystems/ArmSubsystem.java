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
  private LinearInterpolator linearInterpolatorPOS;
  
  
  public ArmSubsystem(ServoSubsystemConfiguration configuration) {
    super(configuration);
    m_armMagnet = new BeamBreak(0);
    linearVision = new LinearInterpolator(interpulationVision);
    linearInterpolatorPOS = new LinearInterpolator(interpulationPOS);
    SmartDashboard.putNumber("armOffset", 0);
    SmartDashboard.putNumber("ArmAngleToSet", 90);
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
    return linearInterpolatorPOS.getInterpolatedValue(limeVal);
  }

  public void setUsingInterpulationPOS(double value) {
    double angle = linearInterpolatorPOS.getInterpolatedValue(value);
    this.setPosition(angle);
  }

  public void Print(){
    setPosition(SmartDashboard.getNumber("ArmAngleToSet", 90));
  }
  //position Interpulation
  double[][] interpulationPOS = 
  {
    {1.183, 57.0},
    {1.48, 51.0},
    {1.779, 47.0},
    {2.041, 44.0},
    {2.30, 40.0},
    {2.60, 39.0},
    {2.92, 36.5},
    {3.29, 34.0},
    {3.51, 33.0},
    {3.95, 31.0},
    {4.15, 30.25},
    {4.48, 29.1},
    {4.55, 28.25},
    {4.71,27.7},
    {5.07, 27.25},
    {5.21, 26.0},
    {5.40, 26.75},
    {5.56,26.5},
    {5.67, 26.4},
    {5.90, 26.0},
    {6.10, 25.8},
    {6.65, 25.4},
    {6.74, 24.9}

  };
  //vision Intepulation
  double[][] interpulationVision = 
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
    {-1.51, 24.9}



  };

  //ty //angle
  //37, 54
  //35.93, 52
  //32.93, 51
  //29.44, 50
  //26.59, 48
  //23.85, 46
  //20.92, 44
  //18.81, 42
  //17.1, 40
  //16.1, 38
  //15.1,38
  //14.1, 37
  //12.95,36.5
  //12.12,36.25
  //11.57,35.5
  //11.14,34.5
  //10.56,34
  //9.87,33.5
  //9.38,32.75
  //8.89,32
  //8.36,31.75
  //7.87,31
  //7.32,31
  //6.90,30.75
  //6.60,30.33
  //6.27,29.5
  //5.87,29.5
  //5.57,29.5
  //5.25,29
  //4.92,28.75
  //4.64,28.75
  //4.24,28.75
  //3.84,27.75
  //3.47,27.25
  //3.21,27.25
  //2.90,27.25
  //2.65,27
  //2.43,27
  //2.24,27
  //2.02,27
  //1.71,27
  //1.59,26.5
  //1.41,26.5
  //1.17,26.5
  //0.97,25.75
  //0.81,25.75
  //0.64,25.75
  //0.43,25.75
  //0.37,25.5
  //0.36,25.5
  //0.21,25
  //





}