package frc.DELib.Subsystems.Swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.DELib.Conversions;
import frc.DELib.Subsystems.Swerve.SwerveUtil.SwerveModuleConstants;

public class SwerveModule {
    /* hardware */
    private TalonFX m_driveMotor; //drive motor
    private TalonFX m_steeringMotor; //steer motor
    private CANcoder m_absoluteEncoder;//absolute Encoder it is a Encoder that remmber the angle the robot wheel was when last time was on

    /* signals */
    /*drive motor signals */
    private final StatusSignal<Double> m_driveMotorPositionSignal;
    private final StatusSignal<Double> m_driveMotorVelocitySignal;
    private final StatusSignal<Double> m_driveMotorSupplyCurrentSignal;
    private final StatusSignal<Double> m_driveMotorClosedLoopErrorSignal;
    /*steer motor signals */
    private final StatusSignal<Double> m_steerMotorPositionSignal;
    private final StatusSignal<Double> m_steerMotorVelocitySignal;
    private final StatusSignal<Double> m_steerMotorSupplyCurrentSignal;
    private final StatusSignal<Double> m_steerMotorClosedLoopErrorSignal;
    /*cancoder signals */
    private final StatusSignal<Double> m_CANCoderAbsolutePositionSignal;
    /* odometry Signals */
    private final BaseStatusSignal[] odometrySignals;

    /* settings */
    private SwerveConstants m_swerveConstants;// the whole swerve constants
    @SuppressWarnings("unused")
    private SwerveModuleConstants m_moduleConstants; //this indavidual constants

    /* drive motor control requests */
    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
    private final VelocityVoltage driveVelocity = new VelocityVoltage(0).withSlot(0);
    /* steer motor control requests */
    private final PositionVoltage anglePosition = new PositionVoltage(0).withSlot(0);
    /*Sysid control requests*/
    private final VoltageOut m_sysidControl = new VoltageOut(0);

    /* others */
    private Rotation2d m_angleOffset = new Rotation2d(); //module angle offset
    private Rotation2d m_lastAngle = new Rotation2d();//last set angle

    public SwerveModule(SwerveModuleConstants swerveModuleConstants, SwerveConstants swerveConstants){
        m_swerveConstants = swerveConstants;
        m_moduleConstants = swerveModuleConstants;

        m_absoluteEncoder = new CANcoder(swerveModuleConstants.absoluteEncoderID, "Canivore");
        m_absoluteEncoder.getConfigurator().apply(swerveConstants.canCoderConfigs());

        m_steeringMotor = new TalonFX(swerveModuleConstants.steeringMotorID, "Canivore");
        m_steeringMotor.getConfigurator().apply(swerveConstants.steerTalonFXConfigs());

        m_driveMotor = new TalonFX(swerveModuleConstants.driveMotorID, "Canivore");
        m_driveMotor.getConfigurator().apply(swerveConstants.driveTalonFXConfigs());
        m_driveMotor.getConfigurator().apply(swerveModuleConstants.slot0Configs);
        m_driveMotor.getConfigurator().setPosition(0.0);

        m_driveMotorPositionSignal = m_driveMotor.getPosition();
        m_driveMotorVelocitySignal = m_driveMotor.getVelocity();
        m_driveMotorSupplyCurrentSignal = m_driveMotor.getSupplyCurrent();
        m_driveMotorClosedLoopErrorSignal = m_driveMotor.getClosedLoopError();

        m_steerMotorPositionSignal = m_steeringMotor.getPosition();
        m_steerMotorVelocitySignal = m_steeringMotor.getVelocity();
        m_steerMotorSupplyCurrentSignal = m_steeringMotor.getSupplyCurrent();
        m_steerMotorClosedLoopErrorSignal = m_steeringMotor.getClosedLoopError();

        m_CANCoderAbsolutePositionSignal = m_absoluteEncoder.getAbsolutePosition();

        odometrySignals = new BaseStatusSignal[4];
        odometrySignals[0] = m_driveMotorVelocitySignal;
        odometrySignals[1] = m_driveMotorPositionSignal;
        odometrySignals[2] = m_steerMotorVelocitySignal;
        odometrySignals[3] = m_steerMotorPositionSignal;

        CanBusProperties(50);

        m_angleOffset = swerveModuleConstants.angleOffset;
    }

    /**
     * @return the speed of the drive motor in mps
     */
    public double getVelocity(){ 
        return m_driveMotor.getVelocity().getValue();
    }

    /**
     * @return the angle of the steering motor in degrees
     */
    public Rotation2d getAngle(){
        return Rotation2d.fromRotations(m_steeringMotor.getPosition().getValue());
    }

    /**
     * @return the abs angle of the steering motor in degrees with offset
     */
    public Rotation2d getAbsAngleWithOffset(){
        return Rotation2d.fromRotations(m_absoluteEncoder.getAbsolutePosition().getValue()).minus(m_angleOffset);
    }

    /**
     * @return the abs angle of the steering motor in degrees
     */
    public Rotation2d getAbsAngle(){
        return Rotation2d.fromRotations(m_absoluteEncoder.getAbsolutePosition().getValue());
    }

    /**
     * @return state of the module (speed and angle)
     */
    public  SwerveModuleState getState(){
        return new SwerveModuleState(getVelocity(), getAngle());
    }

    /**
     * 
     * @return get distance traveled and angle set
     */
    public SwerveModulePosition getModulePosition(){
        return new SwerveModulePosition(
            Conversions.rotationsToMeters(m_driveMotor.getPosition().getValue(), m_swerveConstants.wheelCircumference), 
            getAngle()
        );
    }

    /**
     * sets the drive and steer motors to run with a threshold of 0.03 percent
     * @param desiredState holds the value of the speed and angle of every motor
     * @param openLoop whether the robot uses open loop or closed loop (percenctOutput or velocity)
     */
    public void setState(SwerveModuleState desiredState , boolean openLoop, boolean avoidJittering){
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle);
        if(avoidJittering && Math.abs(desiredState.speedMetersPerSecond) <= (m_swerveConstants.maxSpeed * 0.03)){
            desiredState.angle = m_lastAngle;
        }
        setAngle(desiredState.angle);
        m_lastAngle = desiredState.angle;
        // desiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond * Math.cos(m_steerClosedLoopError.getValueAsDouble() * 2 * Math.PI);
        setSpeed(desiredState , openLoop);
    }

    /**
     * sets the module steering motor netural mode (coast or brake)
     * @param neutralMode
     */
    public void setSteeringNeturalMode(NeutralModeValue neutralMode){
        m_steeringMotor.setNeutralMode(neutralMode);
    }

    /**
     * sets the module angle offset and resets to abolute
     * @param angleOffset
     */
    public void setAngleOffset(Rotation2d angleOffset){
        m_angleOffset = angleOffset;
        resetToAbsolute();
    }

    /**
     * resets the steering motor encoder position using the absolte position given by the CANcoder
     */
    public void resetToAbsolute(){
        m_steeringMotor.setPosition(getAbsAngleWithOffset().getRotations());
    }

    /**
     *  resets the steering motor's encoder position without any monitoring (בקרה)
     */
    public void resetAngle(){                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       
        m_steeringMotor.setPosition(0);
    }

    /** Runs the module with the specified voltage while controlling to zero degrees. */
    public void runCharacterization(double volts) {
        setAngle(Rotation2d.fromRotations(0));
        m_driveMotor.setControl(m_sysidControl.withOutput(volts));
    }

    public void DisableMotors(){
        m_driveMotor.disable();
        m_steeringMotor.disable();
    }

    public void refreshAllSignals(){
        BaseStatusSignal.refreshAll(
        m_driveMotorPositionSignal,
        m_driveMotorVelocitySignal,
        m_driveMotorSupplyCurrentSignal,
        m_driveMotorClosedLoopErrorSignal,
        m_steerMotorPositionSignal,
        m_steerMotorVelocitySignal,
        m_steerMotorSupplyCurrentSignal,
        m_steerMotorClosedLoopErrorSignal,
        m_CANCoderAbsolutePositionSignal);
    }

    public void refreshAllSignalsWithoutOdometrySignals(){
        BaseStatusSignal.refreshAll(
        m_driveMotorSupplyCurrentSignal,
        m_driveMotorClosedLoopErrorSignal,
        m_steerMotorSupplyCurrentSignal,
        m_steerMotorClosedLoopErrorSignal,
        m_CANCoderAbsolutePositionSignal);
    }

    public BaseStatusSignal[] getOdometrySignals(){
        return odometrySignals;
    }

    /**
     *  sets the steering motor's position using pid and a conversion method degrees to ticks
     * @param angle the desired angle of the motor
     */
    private void setAngle(Rotation2d angle){
        m_steeringMotor.setControl(anglePosition.withPosition(angle.getRotations()));
    }

    /**
     * this function dictates the motor whether to use percentOutput to move the motors or velocity with pid.
     *  clamp is used to keep the percentOutput values between -1 and 1
     * @param speedMPS desired speed in mps
     * @param openLoop whether the robot uses open loop or closed loop (percenctOutput or velocity)
     */
    private void setSpeed(SwerveModuleState desiredState , boolean isOpenLoop){
        if(isOpenLoop){
            driveDutyCycle.Output = desiredState.speedMetersPerSecond / m_swerveConstants.maxSpeed;
            m_driveMotor.setControl(driveDutyCycle);
        }
        else {
            driveVelocity.Velocity = Conversions.MPSToRPS(desiredState.speedMetersPerSecond, m_swerveConstants.maxSpeed);
            m_driveMotor.setControl(driveVelocity);
        }
    }

    private void CanBusProperties(int CanBusFrequency){
        BaseStatusSignal.setUpdateFrequencyForAll(50, 
        m_driveMotorPositionSignal,
        m_driveMotorVelocitySignal,
        m_driveMotorSupplyCurrentSignal,
        m_driveMotorClosedLoopErrorSignal,
        m_steerMotorPositionSignal,
        m_steerMotorVelocitySignal,
        m_steerMotorSupplyCurrentSignal,
        m_steerMotorClosedLoopErrorSignal,
        m_CANCoderAbsolutePositionSignal);
        m_driveMotor.optimizeBusUtilization();
        m_steeringMotor.optimizeBusUtilization();
        m_absoluteEncoder.optimizeBusUtilization();
    }
}
