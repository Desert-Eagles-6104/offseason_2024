// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.DELib.Subsystems.PoseEstimator.PoseEstimatorSubsystem;
import frc.DELib.Subsystems.Swerve.SwerveSubsystem;
import frc.DELib.Subsystems.Swerve.SwerveCommands.ResetSwerveModules;
import frc.DELib.Subsystems.Swerve.SwerveCommands.TeleopDrive;
import frc.DELib.Subsystems.Vision.VisionSubsystem;
import frc.DELib.Subsystems.Vision.VisionUtil.CameraSettings;
import frc.DELib.Util.DriverStationController;
import frc.DELib.Util.SwerveAutoBuilder;
import frc.robot.commands.ArmCommands.ArmAngleToDashBoard;
import frc.robot.commands.ArmCommands.ArmChangeNeutralMode;
import frc.robot.commands.ArmCommands.ArmHoming;
import frc.robot.commands.ArmCommands.ArmWithVision;
import frc.robot.commands.IntagrationCommands.Amp;
import frc.robot.commands.IntagrationCommands.Preset;
import frc.robot.commands.IntakeCommnands.IntakeEatUntilHasNote;
import frc.robot.commands.IntakeCommnands.IntakeForTime;
import frc.robot.commands.IntakeCommnands.IntakeGlubGlub;
import frc.robot.commands.IntakeCommnands.IntakeSetPrecent;
import frc.robot.commands.IntakeCommnands.SimpleIntake;
import frc.robot.commands.ShooterCommands.ShooterSetIfHasNote;
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
  private IntakeSubsystem m_intakeSub;
  private ArmSubsystem m_arm;
  private ShooterSubsystem m_shooter;
  private VisionSubsystem m_vision;
  private PoseEstimatorSubsystem m_poseEstimator;
  private SwerveAutoBuilder swerveAutoBuilder;
  public static BooleanSupplier m_isLocalizetion = ()-> false;
  public static BooleanSupplier m_isLocalizetionOmega = () -> false;
  private Trigger m_canShoot;

  public RobotContainer() {
    m_swerve = SwerveSubsystem.createInstance(Constants.Swerve.swerveConstants);
    m_intakeSub = IntakeSubsystem.getInstance();
    m_arm = new ArmSubsystem(Constants.arm.configuration);
    m_shooter = new ShooterSubsystem(Constants.Shooter.configuration);
    m_vision = new VisionSubsystem(new CameraSettings(-0.30821, 0, 0.10689, 0, 15.13, 180.0, true), new CameraSettings(0, 0, 0, 0, 0, 0, false));
    swerveAutoBuilder = new SwerveAutoBuilder(m_swerve);
    m_poseEstimator = new PoseEstimatorSubsystem(m_swerve);
    m_isLocalizetion = driverStationController.LeftSwitch();
    m_isLocalizetionOmega = driverStationController.LeftMidSwitch();
    m_canShoot = new Trigger(() ->(m_shooter.isAtSetpoint() && m_arm.isAtSetpoint() && VisionSubsystem.getTv()));
    SmartDashboard.putData("reset Odometry from limelight", new InstantCommand(() -> PoseEstimatorSubsystem.resetPositionFromCamera()));
    SwerveBinding();
    armBinding();
    shooterBinding();
    intakeBinding();
    presets();
    resets();
  }

  public void disableMotors() {
    m_swerve.disableModules();
    m_arm.disableMotors();
    m_intakeSub.disableMotors();
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

  public void SwerveBinding(){
    SmartDashboard.putData("calibrate Swerve Modules", new ResetSwerveModules(m_swerve).ignoringDisable(true));
    SmartDashboard.putData("reset Odometry", new InstantCommand(() -> m_swerve.resetOdometry(new Pose2d())).ignoringDisable(true));
    m_swerve.setDefaultCommand(new TeleopDrive(m_swerve, drivercontroller, drivercontroller.R2(), drivercontroller.create(), drivercontroller.options(), drivercontroller.R1()));
  }

  public void armBinding(){
    SmartDashboard.putData("Change Arm  NeutralMode", new ArmChangeNeutralMode(m_arm).ignoringDisable(true)); 
    SmartDashboard.putData("reset arm", new InstantCommand(() -> m_arm.resetPosition(9.57)).ignoringDisable(true));
    SmartDashboard.putData("ArmDisableSoftLimit", new InstantCommand(() -> m_arm.ControlSoftLimit(false)).ignoringDisable(true));
    drivercontroller.R1().onTrue(new ArmWithVision(m_arm, m_vision).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    drivercontroller.povUp().onTrue(new ArmAngleToDashBoard(m_arm, m_shooter));
  }

  public void shooterBinding(){
    drivercontroller.R1().onTrue(new ShooterSetIfHasNote(m_shooter, m_intakeSub, 7000));
  }

  public void intakeBinding(){
    drivercontroller.L2().onTrue(new IntakeEatUntilHasNote(m_intakeSub, 0.7, true).andThen(new IntakeGlubGlub(m_intakeSub, true)).andThen(new IntakeEatUntilHasNote(m_intakeSub, 0.5, false)).andThen(new IntakeGlubGlub(m_intakeSub, false)));
    // drivercontroller.L2().onFalse(new IntakeForTime(m_intakeSub,0,0).andThen((new IntakeGlubGlub(m_intakeSub, false))));
    drivercontroller.R2().whileTrue(new IntakeForTime(m_intakeSub, -0.3, 2.0));
    drivercontroller.R1().debounce(0.4).and(m_canShoot).onTrue(new IntakeForTime(m_intakeSub, 0.3, 1.0));
  }

  public void presets(){
    driverStationController.RightYellow().onTrue(new InstantCommand(() -> m_intakeSub.setMotorPrecent(0.3)));
    driverStationController.DownYellow().onTrue(new Preset(m_shooter, m_arm, 9.57, 7000));
    drivercontroller.triangle().onTrue(new Amp(m_intakeSub, m_arm, m_shooter));
  }

  public void resets(){
    drivercontroller.L1().onTrue(new Preset(m_shooter, m_arm, 10, 0));
    drivercontroller.povDown().onTrue(new ArmHoming(m_arm));
  }
}