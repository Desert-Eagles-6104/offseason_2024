// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.DELib.BooleanUtil.LatchedBolean;
import frc.DELib.Sensors.BeamBreak;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  private static IntakeSubsystem m_instance = null;

  private TalonFX m_master;
  private TalonFX m_slave;
  private BeamBreak m_beamBreak;
  private boolean m_hasGamePiece;

  private PositionVoltage m_PositionVoltageRequest = new PositionVoltage(0);

  private TalonFXConfiguration configuration;

  private StatusSignal<Double> m_positionSignal;
  private StatusSignal<Double> m_velocitySignal;
  private StatusSignal<Double> m_closedLoopErrorSignal;
  
  public IntakeSubsystem() {
    //Configuration
    configuration = new TalonFXConfiguration();
    configuration.withMotorOutput(new MotorOutputConfigs().withInverted(Constants.Intake.masterInvert).withDutyCycleNeutralDeadband(0.03));
    configuration.withSlot0(new Slot0Configs().withKS(Constants.Intake.Ks).withKV(0.0).withKA(0.0).withKP(Constants.Intake.Kp).withKI(0.0).withKD(0.0));
    configuration.withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(1));
    configuration.withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLimit(Constants.Intake.supplyCurrentLimit).withSupplyCurrentLimitEnable(true));

    //Master
    m_master = new TalonFX(Constants.Intake.masterID); 
    m_master.getConfigurator().apply(configuration);

    m_positionSignal = m_master.getPosition();
    m_velocitySignal = m_master.getVelocity();
    m_closedLoopErrorSignal = m_master.getClosedLoopError();
    BaseStatusSignal.setUpdateFrequencyForAll(50 ,m_positionSignal, m_velocitySignal, m_closedLoopErrorSignal);

    //Slave
    m_slave = new TalonFX(Constants.Intake.SlaveID);
    m_slave.getConfigurator().apply(configuration);
    m_slave.getConfigurator().apply(new MotorOutputConfigs().withInverted(Constants.Intake.slaveInvert));
    
    //BeamBreak
    m_beamBreak = new BeamBreak(Constants.Intake.beamBrakPort);

  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(m_positionSignal, m_velocitySignal, m_closedLoopErrorSignal);
    m_beamBreak.update();
    m_hasGamePiece = m_beamBreak.get();
    SmartDashboard.putBoolean("HasNote", m_hasGamePiece);
  }

  public void disableMotors(){
    m_master.disable();
    m_slave.disable();
  }

  public double getPosition(){
    return (m_positionSignal.getValueAsDouble());
  }

  public double getVelocity(){
    return m_velocitySignal.getValueAsDouble();
  }

  public boolean hasGamePiece(){
    return m_hasGamePiece;
  }

  public void setMotorPrecent(double precent){
    m_master.set(precent);
    m_slave.set(precent);
  }

  public void setPosition(double position){
    m_master.setControl(m_PositionVoltageRequest.withPosition((getPosition() + position)));
  }

  public static IntakeSubsystem getInstance() {
    if(m_instance == null){
      m_instance = new IntakeSubsystem();
    }  
    return m_instance;
  }
}
