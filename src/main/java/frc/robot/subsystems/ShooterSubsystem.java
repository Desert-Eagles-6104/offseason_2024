// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import java.io.IOException;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.DELib.CSV.CSVReader;
import frc.DELib.Intepulation.LinearInterpolator;
import frc.DELib.Motors.MotorConstants;
import frc.DELib.Motors.PIDContainer;
import frc.DELib.Motors.TalonFXFactory;
import frc.DELib.Subsystems.VelocitySubsystem.VelocitySubsystemConfiguration;

public class ShooterSubsystem extends SubsystemBase{
  /** Creates a new ServoSubsystem. */
  private VelocitySubsystemConfiguration m_configuration;
  
  private TalonFX m_rightMotor;
  private TalonFX m_leftMotor; 

    // Requests
  private MotionMagicVelocityVoltage m_motiongMagicVelocityRequest = new MotionMagicVelocityVoltage(0).withSlot(0);
  private VelocityVoltage m_VelocityVoltageRequest = new VelocityVoltage(0);
  private DutyCycleOut m_dutyCycleRequest = new DutyCycleOut(0);
  private final VoltageOut m_sysidControlRequest = new VoltageOut(0);

  private final StatusSignal<Double> m_positionSignalRight;
  private final StatusSignal<Double> m_velocitySignalRight;
  private final StatusSignal<Double> m_accelerationSignalRight;
  private final StatusSignal<Double> m_supplyCurrentSignalRight;
  private final StatusSignal<Double> m_statorCurrentSignalRight;
  private final StatusSignal<Double> m_closedLoopErrorRight;
  private final StatusSignal<Double> m_appliedVoltageSignalRight;

  private final StatusSignal<Double> m_positionSignalLeft;
  private final StatusSignal<Double> m_velocitySignalLeft;
  private final StatusSignal<Double> m_accelerationSignalLeft;
  private final StatusSignal<Double> m_supplyCurrentSignalLeft;
  private final StatusSignal<Double> m_statorCurrentSignalLeft;
  private final StatusSignal<Double> m_closedLoopErrorLeft;
  private final StatusSignal<Double> m_appliedVoltageSignalLeft;

  private double[][] m_shootingTable;
  private LinearInterpolator linearInterpolator;

  private final double ratioBetweenMotors = 2.0/3.0;  

  private double leftSetpoint = 0;
  private double rightSetpoint = 0;

  private PIDContainer left = new PIDContainer(0.0, 0.068, 0.0, 0.0, 0.0, 0.0, 0.0);  

  public ShooterSubsystem(VelocitySubsystemConfiguration configuration) {
    m_configuration =  configuration;

    TalonFXConfiguration talonConfigurationRight = TalonFXFactory.getDefaultConfig();
    talonConfigurationRight.MotionMagic
    .withMotionMagicCruiseVelocity(configuration.motionMagicCruiseVelocity * configuration.rotationsPerPositionUnit)
    .withMotionMagicAcceleration(configuration.motionMagicAcceleration * configuration.rotationsPerPositionUnit)
    .withMotionMagicJerk(configuration.motionMagicJerk * configuration.rotationsPerPositionUnit);

    talonConfigurationRight.CurrentLimits
    .withStatorCurrentLimitEnable(configuration.enableStatorCurrentLimit)
    .withStatorCurrentLimit(configuration.statorCurrentLimit)
    .withSupplyCurrentLimitEnable(configuration.enableSupplyCurrentLimit)
    .withSupplyCurrentLimit(configuration.supplyCurrentLimit);

    talonConfigurationRight.withSlot0(PIDContainer.toSlot0Configs(configuration.pidContainer));

    talonConfigurationRight.MotorOutput
    .withInverted(InvertedValue.Clockwise_Positive)
    .withNeutralMode(MotorConstants.toNeturalMode(configuration.master.isBrake));

    talonConfigurationRight.Feedback.withSensorToMechanismRatio(configuration.sensorToMechanismRatio);

    m_rightMotor = TalonFXFactory.createTalonFX(configuration.master, talonConfigurationRight);
    
    TalonFXConfiguration talonConfigurationLeft = TalonFXFactory.getDefaultConfig();
    talonConfigurationLeft.MotionMagic
    .withMotionMagicCruiseVelocity(configuration.motionMagicCruiseVelocity * configuration.rotationsPerPositionUnit)
    .withMotionMagicAcceleration(configuration.motionMagicAcceleration * configuration.rotationsPerPositionUnit)
    .withMotionMagicJerk(configuration.motionMagicJerk * configuration.rotationsPerPositionUnit);

    talonConfigurationLeft.CurrentLimits
    .withStatorCurrentLimitEnable(configuration.enableStatorCurrentLimit)
    .withStatorCurrentLimit(configuration.statorCurrentLimit)
    .withSupplyCurrentLimitEnable(configuration.enableSupplyCurrentLimit)
    .withSupplyCurrentLimit(configuration.supplyCurrentLimit);

    talonConfigurationLeft.withSlot0(PIDContainer.toSlot0Configs(configuration.pidContainer));

    talonConfigurationLeft.MotorOutput
    .withInverted(InvertedValue.CounterClockwise_Positive)
    .withNeutralMode(MotorConstants.toNeturalMode(configuration.master.isBrake));

    talonConfigurationLeft.Feedback.withSensorToMechanismRatio(configuration.sensorToMechanismRatio);

    m_leftMotor = TalonFXFactory.createTalonFX(configuration.slaves[0], talonConfigurationLeft);

    // Init signals
    m_positionSignalRight = m_rightMotor.getPosition();
    m_velocitySignalRight = m_rightMotor.getVelocity();
    m_accelerationSignalRight = m_rightMotor.getAcceleration();
    m_supplyCurrentSignalRight = m_rightMotor.getSupplyCurrent();
    m_statorCurrentSignalRight = m_rightMotor.getStatorCurrent();
    m_appliedVoltageSignalRight = m_rightMotor.getMotorVoltage();
    m_closedLoopErrorRight = m_rightMotor.getClosedLoopError();

    m_positionSignalLeft = m_leftMotor.getPosition();
    m_velocitySignalLeft = m_leftMotor.getVelocity();
    m_accelerationSignalLeft = m_leftMotor.getAcceleration();
    m_supplyCurrentSignalLeft = m_leftMotor.getSupplyCurrent();
    m_statorCurrentSignalLeft = m_leftMotor.getStatorCurrent();
    m_appliedVoltageSignalLeft = m_leftMotor.getMotorVoltage();
    m_closedLoopErrorLeft = m_leftMotor.getClosedLoopError();

    BaseStatusSignal.setUpdateFrequencyForAll(50,m_closedLoopErrorRight, m_positionSignalRight, m_velocitySignalRight, m_accelerationSignalRight, m_appliedVoltageSignalRight, m_supplyCurrentSignalRight, m_statorCurrentSignalRight);
    m_rightMotor.optimizeBusUtilization();

    BaseStatusSignal.setUpdateFrequencyForAll(50,m_closedLoopErrorLeft, m_positionSignalLeft, m_velocitySignalLeft, m_accelerationSignalLeft, m_appliedVoltageSignalLeft, m_supplyCurrentSignalLeft, m_statorCurrentSignalLeft);
    m_leftMotor.optimizeBusUtilization();

    linearInterpolator = new LinearInterpolator(interpulation);

    SmartDashboard.putNumber("shooterOffset", 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    BaseStatusSignal.refreshAll(
      m_closedLoopErrorRight,
      m_positionSignalRight, 
      m_velocitySignalRight, 
      m_accelerationSignalRight, 
      m_supplyCurrentSignalRight, 
      m_statorCurrentSignalRight, 
      m_appliedVoltageSignalRight,
      m_closedLoopErrorLeft,
      m_positionSignalLeft, 
      m_velocitySignalLeft, 
      m_accelerationSignalLeft, 
      m_supplyCurrentSignalLeft, 
      m_statorCurrentSignalLeft, 
      m_appliedVoltageSignalLeft);

      SmartDashboard.putNumber("shooter left RPM", getVelocityLeft());
      SmartDashboard.putNumber("shooter right RPM", getVelocityRight());
      SmartDashboard.putBoolean("leftSetpoint", isAtSetpointLeft());
      SmartDashboard.putBoolean("rightSetpoint", isAtSetpointRight());

  }

  public void disableMotors() {
    m_rightMotor.disable();
    m_leftMotor.disable();
  }

  public void runCharacterization(Measure<Voltage> volts) {
    m_rightMotor.setControl(m_sysidControlRequest.withOutput(volts.in(Volts)));
    m_leftMotor.setControl(m_sysidControlRequest.withOutput(volts.in(Volts)));
  }

  public double toRotations(double units) {
    return units * m_configuration.rotationsPerPositionUnit;
  }

  public double fromRotations(double rotations) {
    return rotations / m_configuration.rotationsPerPositionUnit;  
  }

  public void setMotionMagicVelocityWithRatio(double velocity) {
    velocity = velocity + SmartDashboard.getNumber("ShooterOffset", 0);
    rightSetpoint = velocity;
    leftSetpoint = velocity * ratioBetweenMotors;
    m_rightMotor.setControl(m_motiongMagicVelocityRequest.withVelocity(toRotations(velocity)));
    m_leftMotor.setControl(m_motiongMagicVelocityRequest.withVelocity(toRotations(velocity * ratioBetweenMotors)));
  }

  public void setVelocityWithRatio(double velocity) {
    velocity = velocity + SmartDashboard.getNumber("ShooterOffset", 0);
    rightSetpoint = velocity;
    leftSetpoint = velocity * ratioBetweenMotors;
    m_rightMotor.setControl(m_VelocityVoltageRequest.withVelocity(toRotations(velocity)));
    m_leftMotor.setControl(m_VelocityVoltageRequest.withVelocity(toRotations(velocity * ratioBetweenMotors)));
  }

  public void setMotionMagicVelocity(double velocity) {
    velocity = velocity + SmartDashboard.getNumber("ShooterOffset", 0);
    rightSetpoint = velocity;
    leftSetpoint = velocity;
    m_rightMotor.setControl(m_motiongMagicVelocityRequest.withVelocity(toRotations(velocity)));
    m_leftMotor.setControl(m_motiongMagicVelocityRequest.withVelocity(toRotations(velocity)));
  }

  public void setVelocity(double velocity) {
    velocity = velocity + SmartDashboard.getNumber("ShooterOffset", 0);
    rightSetpoint = velocity;
    leftSetpoint = velocity;
    m_rightMotor.setControl(m_VelocityVoltageRequest.withVelocity(toRotations(velocity)));
    m_leftMotor.setControl(m_VelocityVoltageRequest.withVelocity(toRotations(velocity)));
  }

  public void setPrecentOutput(double precent) {
    m_rightMotor.setControl(m_dutyCycleRequest.withOutput(precent));
    m_leftMotor.setControl(m_dutyCycleRequest.withOutput(precent));
  }

  public boolean isAtSetpointRight() {
   return Math.abs(rightSetpoint - getVelocityRight()) < m_configuration.allowableError;
  }

  public boolean isAtSetpointLeft() {
    return Math.abs(leftSetpoint - getVelocityLeft()) < m_configuration.allowableError;
  }

  public boolean isAtSetpoint(){
    return isAtSetpointLeft() && isAtSetpointRight();
  }

  public double getVelocityRight() {
   return  fromRotations(m_velocitySignalRight.getValueAsDouble());
  }

  public double getVelocityLeft() {
   return  fromRotations(m_velocitySignalLeft.getValueAsDouble());
  }

  public void setShootingTable(String Filelocation) {
      try {
        CSVReader reader = new CSVReader(Filelocation);
        m_shootingTable = reader.readAsDouble(2);
        linearInterpolator = new LinearInterpolator(m_shootingTable);
      } catch (IOException e) {

      }
  }

  public double getMotorCurrentRight() {
    return m_rightMotor.getSupplyCurrent().getValueAsDouble();
  }

  public double getMotorCurrentLeft() {
    return m_leftMotor.getSupplyCurrent().getValueAsDouble();
  }

  public void setUsingInterpulation(double value, boolean useMotionMagic) {
    double speed = linearInterpolator.getInterpolatedValue(value);
    if(useMotionMagic){
      setMotionMagicVelocityWithRatio(speed);
    }
    else{
      setVelocityWithRatio(speed);
    }
  }

  public double getInterpulationVelocity(double value) {
    return linearInterpolator.getInterpolatedValue(value);
  }

  double[][] interpulation = 
  {
    {1,4000},
    {3,4200},
    {6,4400},
    {8,4600},
    {10,4800},
    {12,5000},
    {14,5200},
    {16,5400},
    {18,5600},
    {20,5800}
  };
}