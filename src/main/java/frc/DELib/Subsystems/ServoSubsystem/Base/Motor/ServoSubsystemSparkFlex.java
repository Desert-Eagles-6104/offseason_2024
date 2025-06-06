// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.DELib.Subsystems.ServoSubsystem.Base.Motor;

import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.CANSparkFlex;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.SoftLimitDirection;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.DELib.Subsystems.ServoSubsystem.ServoSubsystemConfiguration;
import frc.DELib.Subsystems.ServoSubsystem.Base.IServoSubsystemBase;

public class ServoSubsystemSparkFlex extends SubsystemBase implements IServoSubsystemBase{
  /** Creates a new ServoSubsystem. */
  private ServoSubsystemConfiguration m_configuration;

  private CANSparkFlex master;
  private CANSparkFlex[] slaveFlex;
  private SimpleMotorFeedforward feedforward;

  private double setpoint = -99999;
  
  public ServoSubsystemSparkFlex(ServoSubsystemConfiguration configuration) {
    m_configuration =  configuration;

    master = ServoSubsystemMotorFactory.createSparkFlex(configuration);
    if(configuration.slaves != null){
      slaveFlex = ServoSubsystemMotorFactory.createSlaveSparkFlex(configuration, master);
    }

    feedforward = new SimpleMotorFeedforward(configuration.pidContainerSlot0.kS, configuration.pidContainerSlot0.kV, configuration.pidContainerSlot0.kA); 

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber(m_configuration.subsystemName + "Position", getPosition());
  }

  @Override
  public void resetSubsystemToInitialState() {
    resetPosition(m_configuration.homePosition);
  }

  @Override
  public void disableMotors() {
    master.disable();
  }

  @Override
  public void runCharacterization(Measure<Voltage> volts) {
    master.setVoltage(volts.in(Volts));
  }

  @Override
  public double toRotations(double units) {
    return units * m_configuration.rotationsPerPositionUnit;
  }

  @Override
  public double fromRotations(double rotations) {
    return rotations / m_configuration.rotationsPerPositionUnit;  
  }

  @Override
  public void setMotionMagicPosition(double position) {
    setpoint = position;
    master.getPIDController().setFF(feedforward.calculate(position));
    master.getPIDController().setReference(toRotations(position), ControlType.kSmartVelocity);
  }

  @Override
  public void ControlSoftLimit(boolean enableSoftLimit) {
    master.enableSoftLimit(SoftLimitDirection.kForward ,enableSoftLimit);
    master.enableSoftLimit(SoftLimitDirection.kReverse ,enableSoftLimit);
  }

  @Override
  public void setPosition(double position) {
    setpoint = position;
    master.getPIDController().setFF(feedforward.calculate(position));
    master.getPIDController().setReference(toRotations(position), ControlType.kVelocity);
  }

  @Override
  public void setPrecentOutput(double precent) {
    master.set(precent);
  }



  @Override
  public void resetPosition(double position) {
    master.getEncoder().setPosition(position);
    if(slaveFlex != null){
      for(int i = 0; i < slaveFlex.length; i++){
        slaveFlex[i].getEncoder().setPosition(position);
      }
    }
  }

  @Override
  public boolean isAtSetpoint() {
    return inRange(setpoint - getPosition(), m_configuration.allowableError);
  }

  @Override
  public double getPosition() {
    return master.getEncoder().getPosition();
  }

  @Override
  public double getMotorCurrent() {
    return master.getOutputCurrent();
  }

  @Override
  public double getClosedLoopError() {
    return 0;
  }

  @Override
  public double getVelocity() {
    return master.getEncoder().getVelocity();
  }

  public boolean inRange(double v, double min, double max){
    return v >= min && v <= max;
  }

  /**
   * 
   * @param v
   * @param magnitude
   * @return 
   */
  public boolean inRange(double v, double magnitude){
    return inRange(v, -magnitude, magnitude);
  }

  @Override
  public void changeNeutralMode(NeutralModeValue NeutralMode) {
  
  }
}
