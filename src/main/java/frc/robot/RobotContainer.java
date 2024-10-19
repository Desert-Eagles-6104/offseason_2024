// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.DELib.Subsystems.PoseEstimator.PoseEstimatorSubsystem;
import frc.DELib.Subsystems.ServoSubsystem.Commands.ServoSubsystemManualControl;
import frc.DELib.Subsystems.Swerve.SwerveSubsystem;
import frc.DELib.Subsystems.Swerve.SwerveCommands.ResetSwerveModules;
import frc.DELib.Subsystems.Swerve.SwerveCommands.RotateToTarget;
import frc.DELib.Subsystems.Swerve.SwerveCommands.SwerveDisableMotors;
import frc.DELib.Subsystems.Swerve.SwerveCommands.TeleopDrive;
import frc.DELib.Subsystems.Swerve.SwerveUtil.DriveAssistAuto;
import frc.DELib.Subsystems.Vision.VisionSubsystem;
import frc.DELib.Subsystems.Vision.VisionUtil.CameraSettings;
import frc.DELib.Util.DriverStationController;
import frc.DELib.Util.SwerveAutoBuilder;
import frc.robot.commands.ArmCommands.ArmChangeNeutralMode;
import frc.robot.commands.ArmCommands.ArmHoming;
import frc.robot.commands.ArmCommands.ArmSetPosition;
import frc.robot.commands.ArmCommands.ArmWithVision;
import frc.robot.commands.ArmCommands.DisableArm;
import frc.robot.commands.IntagrationCommands.Amp;
import frc.robot.commands.IntagrationCommands.AutoShoot;
import frc.robot.commands.IntagrationCommands.Preset;
import frc.robot.commands.IntagrationCommands.ResetAllSubsystems;
import frc.robot.commands.IntakeCommnands.DisableIntake;
import frc.robot.commands.IntakeCommnands.IntakeEatUntilHasNote;
import frc.robot.commands.IntakeCommnands.IntakeForTime;
import frc.robot.commands.IntakeCommnands.IntakeGlubGlub;
import frc.robot.commands.IntakeCommnands.IntakeSetPrecent;
import frc.robot.commands.ShooterCommands.DisableShooter;
import frc.robot.commands.ShooterCommands.ShooterSetIfHasNote;
import frc.robot.commands.ShooterCommands.ShooterSetVelocity;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private CommandPS5Controller drivercontroller = new CommandPS5Controller(0);
  private CommandPS5Controller operatorController = new CommandPS5Controller(1);
  private DriverStationController driverStationController = new DriverStationController(2);
  private SwerveSubsystem m_swerve;
  private IntakeSubsystem m_intake;
  private ArmSubsystem m_arm;
  private ShooterSubsystem m_shooter;
  private VisionSubsystem m_vision;
  private PoseEstimatorSubsystem m_poseEstimator;
  private SwerveAutoBuilder swerveAutoBuilder;
  public static BooleanSupplier m_isLocalizetion = ()-> false;
  public static BooleanSupplier m_isLocalizetionOmega = () -> false;
  private Trigger m_firstBeamBreakSees;
  private Trigger m_firstBeamBreakDontSees;
  private Trigger m_canShoot;
  private BooleanSupplier m_DisableDelivery;
  private BooleanSupplier m_highDeliveryMiddlePreset = () -> false;
  private BooleanSupplier m_lowDeliveryMiddlePreset = () -> false;
  private BooleanSupplier m_highDeliveryWingPreset = () -> false;
  private BooleanSupplier m_inMyWing = () -> false;
  


  public RobotContainer() {
    m_swerve = SwerveSubsystem.createInstance(Constants.Swerve.swerveConstants);
    m_intake = IntakeSubsystem.getInstance();
    m_arm = new ArmSubsystem(Constants.arm.configuration);
    m_shooter = new ShooterSubsystem(Constants.Shooter.configuration);
    m_vision = new VisionSubsystem(new CameraSettings(-0.30821, 0, 0.10689, 0, 15.13, 180.0, true), new CameraSettings(0, 0, 0, 0, 0, 0, false));
    swerveAutoBuilder = new SwerveAutoBuilder(m_swerve);
    m_poseEstimator = new PoseEstimatorSubsystem(m_swerve);
    m_isLocalizetion = driverStationController.LeftSwitch().negate();
    m_isLocalizetionOmega = driverStationController.LeftMidSwitch().negate();
    m_canShoot = new Trigger(() ->(m_shooter.isAtSetpoint() && m_arm.isAtSetpoint())); //TODO: add isCentered 
    m_DisableDelivery = driverStationController.RightMidSwitch().negate();
    m_firstBeamBreakSees =  new Trigger(() -> m_intake.firstBeamBreak());
    m_firstBeamBreakDontSees = new Trigger(() -> !m_intake.firstBeamBreak());
    m_highDeliveryMiddlePreset = () -> PoseEstimatorSubsystem.isAtFeederSide() && PoseEstimatorSubsystem.notInAnyWing();
    m_lowDeliveryMiddlePreset = () -> !PoseEstimatorSubsystem.isAtFeederSide() && PoseEstimatorSubsystem.notInAnyWing();
    m_highDeliveryWingPreset = () -> PoseEstimatorSubsystem.inEnenmyWing() && PoseEstimatorSubsystem.isAtFeederSide();
    m_inMyWing = () -> PoseEstimatorSubsystem.inMyWing();
    SmartDashboard.putData("reset Odometry from limelight", new InstantCommand(() -> PoseEstimatorSubsystem.resetPositionFromCamera()));
    SwerveBinding();
    armBinding();
    intakeBinding();
    presets();
    shotState();
    resets();
    auto();
    driverStationController.LeftBlue().onTrue(new RotateToTarget(m_swerve));
  }

  public void disableMotors() {
    m_swerve.disableModules();
    m_arm.disableMotors();
    m_intake.disableMotors();
    m_shooter.disableMotors();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * 
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return swerveAutoBuilder.getAuto();
  }

  public void auto(){
    //add commands 
    swerveAutoBuilder.addCommand("InatkeUntilHasNote", new IntakeEatUntilHasNote(m_intake, 0.8, true).withTimeout(2));
    swerveAutoBuilder.addCommand("InatkeUntilHasNoteSecondBeamBreak", new IntakeEatUntilHasNote(m_intake, 0.4, false).withTimeout(2));
    swerveAutoBuilder.addCommand("FullIntake", new IntakeEatUntilHasNote(m_intake, 0.7, true).andThen(new IntakeGlubGlub(m_intake, true)).andThen(new IntakeEatUntilHasNote(m_intake, 0.5, false)).andThen(new IntakeGlubGlub(m_intake, false)).withTimeout(2));
    swerveAutoBuilder.addCommand("ShortIntake", new IntakeEatUntilHasNote(m_intake, 0.5, false).andThen(new IntakeGlubGlub(m_intake, false)).withTimeout(2));
    swerveAutoBuilder.addCommand("IntakeDownGlubGlub", ((new IntakeGlubGlub(m_intake, true)).andThen(new IntakeEatUntilHasNote(m_intake, 0.4, false))).withTimeout(2));
    swerveAutoBuilder.addCommand("ArmWithVision", new ArmWithVision(m_arm));
    swerveAutoBuilder.addCommand("Shoot", new ShooterSetVelocity(m_shooter, 7000));
    swerveAutoBuilder.addCommand("AutoShoot", new ParallelDeadlineGroup(new AutoShoot(m_shooter, m_arm, m_intake), new ArmWithVision(m_arm), new ShooterSetVelocity(m_shooter, 7000)));
    swerveAutoBuilder.addCommand("ArmHoming", new ArmHoming(m_arm));
    swerveAutoBuilder.addCommand("DisableArm", new DisableArm(m_arm));
    swerveAutoBuilder.addCommand("DisableShooter", new DisableShooter(m_shooter));
    swerveAutoBuilder.addCommand("DisableIntake", new DisableIntake(m_intake));
    swerveAutoBuilder.addCommand("ResetAll", new ResetAllSubsystems(m_shooter,m_intake,m_arm));
    swerveAutoBuilder.addCommand("RotateToSpeaker", new RotateToTarget(m_swerve));
    swerveAutoBuilder.addCommand("DisableSwerveMotors", new SwerveDisableMotors(m_swerve));
    swerveAutoBuilder.addCommand("IntakePrecent", new IntakeSetPrecent(m_intake,0.8));
    swerveAutoBuilder.addCommand("DriveAssist", new DriveAssistAuto(m_swerve));
    swerveAutoBuilder.addCommand("EnableOffset", new InstantCommand(() -> SmartDashboard.putNumber("ArmAngleOffset", 1)));
    swerveAutoBuilder.addCommand("DisableOffset", new InstantCommand(() -> SmartDashboard.putNumber("ArmAngleOffset", 0.75)));
    swerveAutoBuilder.addCommand("ArmSetIntakePos", new ArmSetPosition(m_arm, 25, true));
    //last
    swerveAutoBuilder.buildAutos();
  }

  public void SwerveBinding(){
    SmartDashboard.putData("calibrate Swerve Modules", new ResetSwerveModules(m_swerve).ignoringDisable(true));
    m_swerve.setDefaultCommand(new TeleopDrive(m_swerve, drivercontroller, drivercontroller.R2(), drivercontroller.create(), drivercontroller.options(), drivercontroller.R1(), drivercontroller.L2()));
  }

  public void armBinding(){
    SmartDashboard.putData("Change Arm  NeutralMode", new ArmChangeNeutralMode(m_arm).ignoringDisable(true)); 
    SmartDashboard.putData("reset arm", new InstantCommand(() -> m_arm.resetPosition(9.57)).ignoringDisable(true));
    SmartDashboard.putData("ArmDisableSoftLimit", new InstantCommand(() -> m_arm.ControlSoftLimit(false)).ignoringDisable(true));
    operatorController.cross().onTrue(new ServoSubsystemManualControl(m_arm, ()-> operatorController.getRightY()));
  }

  public void shotState(){
    drivercontroller.R1().and(m_inMyWing).onTrue(new ArmWithVision(m_arm).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    drivercontroller.R1().and(m_inMyWing).onTrue(new ShooterSetIfHasNote(m_shooter, m_intake, 7000));

    drivercontroller.R1().and(m_lowDeliveryMiddlePreset).and(m_DisableDelivery).onTrue(new Preset(m_shooter, m_arm, 35, 3500));//low middle preset
    drivercontroller.R1().and(m_highDeliveryMiddlePreset).and(m_DisableDelivery).onTrue(new Preset(m_shooter, m_arm, 53, 5000));//high middle preset
    drivercontroller.R1().and(m_highDeliveryWingPreset).and(m_DisableDelivery).onTrue(new Preset(m_shooter, m_arm, 63, 6000));//high wing preset
  
    drivercontroller.triangle().onTrue(new Amp(m_intake, m_arm, m_shooter));
  }

  public void intakeBinding(){
    drivercontroller.L2().onTrue(new ArmHoming(m_arm));
    (m_firstBeamBreakSees).and(drivercontroller.L2()).onTrue(new IntakeEatUntilHasNote(m_intake, 0.5, false).andThen(new IntakeGlubGlub(m_intake, false)));
    (m_firstBeamBreakDontSees).and(drivercontroller.L2()).onTrue(new IntakeEatUntilHasNote(m_intake, 0.7, true).andThen(new IntakeGlubGlub(m_intake, true)).andThen(new IntakeEatUntilHasNote(m_intake, 0.5, false)).andThen(new IntakeGlubGlub(m_intake, false)));
    drivercontroller.R2().whileTrue(new IntakeForTime(m_intake, -0.3, 2.0));
    drivercontroller.R1().debounce(0.4).and(m_canShoot).onTrue(new IntakeForTime(m_intake, 0.8, 1.0).andThen(new WaitCommand(0.5)).andThen(new DisableShooter(m_shooter)).andThen(new ArmHoming(m_arm)));
    operatorController.R1().debounce(0.4).and(m_canShoot).onTrue(new IntakeForTime(m_intake, 0.8, 1.0).andThen(new WaitCommand(0.5)).andThen(new DisableShooter(m_shooter)).andThen(new ArmHoming(m_arm)));
  }

  public void presets(){
    driverStationController.RightYellow().onTrue(new InstantCommand(() -> m_intake.setMotorPrecent(0.8)));
    operatorController.cross().onTrue(new Preset(m_shooter, m_arm, 55, 5000)); //close to speaker preset
    operatorController.square().onTrue(new Preset(m_shooter, m_arm, 36, 6000)); //amp preset
    operatorController.circle().onTrue(new Preset(m_shooter, m_arm, 36, 6000)); //trass preset
    operatorController.triangle().onTrue(new Preset(m_shooter, m_arm, 53, 5000)); //delivary preset
  }

  public void resets(){
    drivercontroller.L1().onTrue(new ResetAllSubsystems(m_shooter, m_intake, m_arm));
    operatorController.L1().onTrue(new ResetAllSubsystems(m_shooter, m_intake, m_arm));
    drivercontroller.povDown().onTrue(new ArmSetPosition(m_arm, 10, true));
  }
}