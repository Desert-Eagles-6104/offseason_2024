// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.DELib.Subsystems.VelocitySubsystem.Base.Motor;

import static edu.wpi.first.units.Units.Volts;

import java.io.IOException;
import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkFlex;
import com.ctre.phoenix6.SignalLogger;
import com.revrobotics.CANSparkBase.ControlType;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.DELib.CSV.CSVReader;
import frc.DELib.Intepulation.LinearInterpolator;
import frc.DELib.Subsystems.VelocitySubsystem.VelocitySubsystemConfiguration;
import frc.DELib.Subsystems.VelocitySubsystem.Base.IVelocitySubsystemBase;

public class VelocitySubsystemSparkFlex extends SubsystemBase implements IVelocitySubsystemBase{
  /** Creates a new ServoSubsystem. */
  private VelocitySubsystemConfiguration m_configuration;

  private CANSparkFlex master;
  @SuppressWarnings("unused")
  private CANSparkFlex[] slaveFlex;
  private SimpleMotorFeedforward feedforward;

  private double setpoint = -99999;

  private double[][] m_shootingTable;
  private LinearInterpolator linearInterpolator;
  
  public VelocitySubsystemSparkFlex(VelocitySubsystemConfiguration configuration) {
    m_configuration =  configuration;

    master = VelocitySubsystemMotorFactory.createSparkFlex(configuration);
    if(configuration.slaves != null){
      slaveFlex = VelocitySubsystemMotorFactory.createSlaveSparkFlex(configuration, master);
    }

    feedforward = new SimpleMotorFeedforward(configuration.pidContainer.kS, configuration.pidContainer.kV, configuration.pidContainer.kA); 
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber(m_configuration.subsystemName + " Velocity", getVelocity());
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
  public void setMotionMagicVelocity(double velocity) {
    master.getPIDController().setFF(feedforward.calculate(velocity));
    master.getPIDController().setReference(toRotations(velocity), ControlType.kSmartVelocity);
  }

  @Override
  public void setVelocity(double velocity) {
    master.getPIDController().setFF(feedforward.calculate(velocity));
    master.getPIDController().setReference(toRotations(velocity), ControlType.kVelocity);
  }

  @Override
  public void setPrecentOutput(double precent) {
    master.set(precent);
  }

  @Override
  public boolean isAtSetpoint() {
    return inRange(setpoint - getVelocity(), m_configuration.allowableError);
  }

  @Override
  public void setShootingTable(String Filelocation) {
      try {
        CSVReader reader = new CSVReader(Filelocation);
        m_shootingTable = reader.readAsDouble(2);
        linearInterpolator = new LinearInterpolator(m_shootingTable);
      } catch (IOException e) {

      }
  }

  @Override
  public double getVelocity() {
    return master.getEncoder().getVelocity();
  }

  @Override
  public void log(BooleanSupplier startLog) {
     if(startLog.getAsBoolean()){
        SignalLogger.writeDouble("motorVoltage" , master.getBusVoltage());
        SignalLogger.writeDouble("motorPosition", master.getEncoder().getPosition());
        SignalLogger.writeDouble("motorVelocity", getVelocity());
    }
  }

  @Override
  public double getMotorCurrent() {
    return master.getOutputCurrent();
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
  public void setUsingInterpulation(double value, boolean useMotionMagic) {
    double speed = linearInterpolator.getInterpolatedValue(value);
    if(useMotionMagic){
      setMotionMagicVelocity(speed);
    }
    else{
      setVelocity(speed);
    }
  }

  @Override
  public double getInterpulationVelocity(double value) {
    return linearInterpolator.getInterpolatedValue(value);
  }
}