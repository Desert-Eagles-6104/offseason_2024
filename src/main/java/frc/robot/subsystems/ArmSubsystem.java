// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.DELib.Intepulation.LinearInterpolator;
import frc.DELib.Sensors.BeamBreak;
import frc.DELib.Subsystems.ServoSubsystem.ServoSubsystemConfiguration;
import frc.DELib.Subsystems.ServoSubsystem.Base.Motor.ServoSubsystemTalon;

public class ArmSubsystem extends ServoSubsystemTalon {
  private BeamBreak MAGNETboI;
  private boolean magnetState = false;
  private LinearInterpolator linearInterpolator;
  private LinearFilter m_filterInterpulation;
  
  public ArmSubsystem(ServoSubsystemConfiguration configuration) {
    super(configuration);
    MAGNETboI = new BeamBreak(0);
    linearInterpolator = new LinearInterpolator(interpulation);
    m_filterInterpulation = LinearFilter.movingAverage(4);
    SmartDashboard.putNumber("armOffset", 0);
  }

  @Override
  public void setMotionMagicPosition(double position) {
    super.setMotionMagicPosition(position + SmartDashboard.getNumber("armOffset", 0));
  }

  @Override
  public void periodic() {
    super.periodic();
    magnetState = magnetUpdate();
    SmartDashboard.putBoolean("magnetcontact", magnetState);
  }

  public boolean magnetUpdate(){
    MAGNETboI.update();
    return MAGNETboI.get();
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

  public void setUsingInterpulation(double value) {
    double angle = m_filterInterpulation.calculate(linearInterpolator.getInterpolatedValue(value));
    setMotionMagicPosition(angle);
  }

  double[][] interpulation = 
  {
    {7.77, 30.41015625},
    {8.6, 31.376953125},
    {9.6, 32.080078125},
    {10.6767, 33.486328125},
    {11.110023384094239, 34.5},
    {12.12, 35.859375},
    {13.83, 36.03515625},
    {15.9, 37.123046875},
    {19.0, 39.314453125},
    {21.4, 43.330078125},
    {22.6, 44},
    {24.3, 47.8125},
    {27.4, 46.58203125},
    {31.94, 48.603515625},
    {37.85, 55.107421875}
  };

  //ty //angle
  // 37.85, 55.107421875
  // 31.94, 48.603515625
  // 27.4, 46.58203125
  // 24.3, 47.8125
  // 21.4, 43.330078125
  // 19.0, 39.314453125
  // 15.9, 37.123046875
  // 13.83, 36.03515625
  // 12.12, 35.859375
  // 10.6767, 33.486328125
  // 9.6, 32.080078125
  // 8.6, 31.376953125
  // 7.77, 30.41015625
}