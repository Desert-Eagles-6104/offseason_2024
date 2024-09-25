// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.DELib.Sensors.BeamBreak;
import frc.DELib.Subsystems.ServoSubsystem.ServoSubsystemConfiguration;
import frc.DELib.Subsystems.ServoSubsystem.Base.Motor.ServoSubsystemTalon;

public class ArmSubsystem extends ServoSubsystemTalon {
  private BeamBreak MAGNETboI;
  private boolean magnetState = false;
  
  public ArmSubsystem(ServoSubsystemConfiguration configuration) {
    super(configuration);
    MAGNETboI = new BeamBreak(8);
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
}