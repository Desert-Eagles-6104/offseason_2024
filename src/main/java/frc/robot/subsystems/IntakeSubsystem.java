// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.DELib.Motors.MotorConstants;
import frc.DELib.Motors.TalonFXFactory;
import frc.DELib.Sensors.BeamBreak;

public class IntakeSubsystem extends SubsystemBase {
  private static IntakeSubsystem m_instance = null;

  private TalonFX m_master;
  private TalonFX m_slave;
  private BeamBreak m_BB;
  private boolean m_hasGamePiece;

  private DutyCycleOut m_dutyCycleRequest = new DutyCycleOut(0);
  private PositionVoltage m_PositionVoltageRequest = new PositionVoltage(0);

  private TalonFXConfigurator configurator;

  private StatusSignal<Double> m_positionSignal;
  private StatusSignal<Double> m_velocitySignal;
  private StatusSignal<Double> m_closedLoopErrorSignal;
  
  public IntakeSubsystem() {
    m_master = TalonFXFactory.createTalonFX(new MotorConstants(55, "rio", false, true), true); //TODO: add id
    configurator = m_master.getConfigurator();
    configurator.apply(new Slot0Configs().withKS(0.0).withKV(0.0).withKA(0.0).withKP(0.0).withKI(0.0).withKD(0.0));
    configurator.apply(new FeedbackConfigs().withSensorToMechanismRatio(0.0));
    m_positionSignal = m_master.getPosition();
    m_velocitySignal = m_master.getVelocity();
    m_closedLoopErrorSignal = m_master.getClosedLoopError();
    BaseStatusSignal.setUpdateFrequencyForAll(50 ,m_positionSignal, m_velocitySignal, m_closedLoopErrorSignal);
    m_slave = TalonFXFactory.createSlaveTalon(new MotorConstants(54, "rio", true , true), 55, false); //TODO: add id
    m_BB = new BeamBreak(0);
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(m_positionSignal, m_velocitySignal, m_closedLoopErrorSignal);
    m_BB.update();
    m_hasGamePiece = m_BB.get();
  }

  public void disableMotors(){
    m_master.disable();
    m_slave.disable();
  }

  public double getPosition(){
    return m_positionSignal.getValueAsDouble();
  }

  public double getVelocity(){
    return m_velocitySignal.getValueAsDouble();
  }

  public boolean hasGamePiece(){
    return m_hasGamePiece;
  }

  public void setMotorPrecent(double precent){
    m_master.setControl(m_dutyCycleRequest.withOutput(precent));
  }

  public void setPosition(double position){
    m_master.setControl(m_PositionVoltageRequest.withPosition(getPosition() + position));
  }

  public static IntakeSubsystem getInstance() {
    if(m_instance == null){
      m_instance = new IntakeSubsystem();
    }  
    return m_instance;
  }
}