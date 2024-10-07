// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.DELib.Subsystems.Swerve.SwerveSubsystem;
import frc.DELib.Subsystems.Swerve.SwerveCommands.ResetSwerveModules;
import frc.DELib.Subsystems.Swerve.SwerveCommands.TeleopDrive;
import frc.DELib.Subsystems.Vision.VisionSubsystem;
import frc.DELib.Subsystems.Vision.VisionUtil.CameraSettings;
import frc.DELib.Util.DriverStationController;
import frc.DELib.Util.SwerveAutoBuilder;
import frc.robot.commands.ArmCommands.ArmChangeNeutralMode;
import frc.robot.commands.ArmCommands.ArmHoming;
import frc.robot.commands.ArmCommands.ArmWithVision;
import frc.robot.commands.IntagrationCommands.Amp;
import frc.robot.commands.IntagrationCommands.Preset;
import frc.robot.commands.IntakeCommnands.IntakeForTime;
import frc.robot.commands.IntakeCommnands.SimpleIntake;
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
  private IntakeSubsystem m_intakeSub;
  private ArmSubsystem m_arm;
  private ShooterSubsystem m_shooter;
  private VisionSubsystem m_vision;
  private SwerveAutoBuilder swerveAutoBuilder;

  public RobotContainer() {
    m_swerve = SwerveSubsystem.createInstance(Constants.Swerve.swerveConstants);
    m_intakeSub = IntakeSubsystem.getInstance();
    m_arm = new ArmSubsystem(Constants.arm.configuration);
    m_shooter = new ShooterSubsystem(Constants.Shooter.configuration);
    m_vision = new VisionSubsystem(new CameraSettings(-0.30821, 0, 0.10689, 0, 15.13, 180.0, true), new CameraSettings(0, 0, 0, 0, 0, 0, false));
    swerveAutoBuilder = new SwerveAutoBuilder(m_swerve);
    SwerveBinding();
    armBinding();
    shooterBinding();
    intakeBinding();
    presets();
    resets();
    // drivercontroller.triangle().onTrue(new setArmPoseDashboard(m_arm));
    // drivercontroller.triangle().onTrue(new ShooterSetVelocity(m_shooter, 7000));
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
    m_swerve.setDefaultCommand(new TeleopDrive(m_swerve, m_vision, drivercontroller, drivercontroller.R2(), drivercontroller.create(), drivercontroller.options(), drivercontroller.R1()));
  }

  public void armBinding(){
    SmartDashboard.putData("Change Arm  NeutralMode", new ArmChangeNeutralMode(m_arm).ignoringDisable(true));
    SmartDashboard.putData("reset arm", new InstantCommand(() -> m_arm.resetPosition(9.57)).ignoringDisable(true));
    SmartDashboard.putData("ArmDisableSoftLimit", new InstantCommand(() -> m_arm.ControlSoftLimit(false)).ignoringDisable(true));
    drivercontroller.R1().onTrue(new ArmWithVision(m_arm, m_vision));
  }

  public void shooterBinding(){
    drivercontroller.R1().onTrue(new ShooterSetVelocity(m_shooter, 7000));
  }

  public void intakeBinding(){
    drivercontroller.L2().whileTrue(new SimpleIntake(m_intakeSub));
    drivercontroller.R2().whileTrue(new IntakeForTime(m_intakeSub, -0.3, 2.0));
    drivercontroller.R1().debounce(0.4).onTrue(new IntakeForTime(m_intakeSub, 0.3, 3));
  }

  public void presets(){
    driverStationController.RightYellow().onTrue(new InstantCommand(() -> m_intakeSub.setMotorPrecent(0.3)));
    driverStationController.DownYellow().onTrue(new Preset(m_shooter, m_arm, 9.57, 7000));
    driverStationController.UpBlue().onTrue(new Amp(m_intakeSub, m_arm, m_shooter));
  }

  public void resets(){
    operatorController.L1().onTrue(new Preset(m_shooter, m_arm, 10, 0));
    drivercontroller.povDown().onTrue(new ArmHoming(m_arm));
  }
}