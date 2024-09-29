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
  private BeamBreak MAGNETboI;
  private boolean magnetState = false;
  private LinearInterpolator linearInterpolator;
  
  public ArmSubsystem(ServoSubsystemConfiguration configuration) {
    super(configuration);
    MAGNETboI = new BeamBreak(8);
    linearInterpolator = new LinearInterpolator(interpulation);
  }

  @Override
  public void periodic() {
    super.periodic();
    magnetState = magnetUpdate();
    SmartDashboard.putBoolean("magnetcontact", magnetState);
  }

  public boolean magnetUpdate(){
    MAGNETboI.update();
    return  MAGNETboI.get();
  }

  public boolean getMagnetState(){
    return magnetState;
  }

  public double getSetpoint(){
    return setpoint;
  }

  public double getInterpulatedAngle(double limeVal){
    return linearInterpolator.getInterpolatedValue(limeVal);
  }

  double[][] interpulation = 
  {
    {1,15.0},
    {3,20},
    {6,25},
    {8,30},
    {10,35},
    {12,40},
    {14,45},
    {16,50},
    {18,55},
    {20,60}
  };
}