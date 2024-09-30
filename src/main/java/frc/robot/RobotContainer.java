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
import frc.DELib.Subsystems.Swerve.SwerveCommands.SwerveSysidCommands;
import frc.DELib.Subsystems.Swerve.SwerveCommands.TeleopDrive;
import frc.DELib.Subsystems.Vision.VisionSubsystem;
import frc.DELib.Subsystems.Vision.VisionUtil.CameraSettings;
import frc.DELib.Util.DriverStationController;
import frc.DELib.Util.SwerveAutoBuilder;
import frc.robot.commands.ArmCommands.ArmChangeNeutralMode;
import frc.robot.commands.ArmCommands.ArmHoming;
import frc.robot.commands.ArmCommands.ArmWithVision;
import frc.robot.commands.IntagrationCommands.Amping;
import frc.robot.commands.IntagrationCommands.Preset;
import frc.robot.commands.IntakeCommnands.IntakeEatNote;
import frc.robot.commands.ShooterCommands.ShooterWithVision;
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
  private SwerveSysidCommands swerveSysidCommands;

  public RobotContainer() {
    m_swerve = SwerveSubsystem.createInstance(Constants.Swerve.swerveConstants);
    m_intakeSub = IntakeSubsystem.getInstance();
    m_arm = new ArmSubsystem(Constants.arm.configuration);
    m_shooter = new ShooterSubsystem(Constants.Shooter.configuration);
    m_vision = new VisionSubsystem(new CameraSettings(0.30821, 0, 0.10689, 0, 15.13, 180.0, true), new CameraSettings(0, 0, 0, 0, 0, 0, false));
    swerveAutoBuilder = new SwerveAutoBuilder(m_swerve);
    SwerveBinding();
    armBinding();
    shooterBinding();
    intakeBinding();
    presets();
    resets();
    swerveSysidCommands =new SwerveSysidCommands(m_swerve);
    driverStationController.LeftSwitch().onTrue(swerveSysidCommands.fullSysidRun());
  }

  public void disableMotors() {
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
    // An example command will be run in autonomous
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
    drivercontroller.circle().onTrue(new InstantCommand(() -> m_arm.setMotionMagicPosition(90)));
    drivercontroller.square().onTrue(new InstantCommand(() -> m_arm.setMotionMagicPosition(45)));
    drivercontroller.PS().onTrue(new InstantCommand(() -> m_arm.setMotionMagicPosition(Constants.arm.configuration.homePosition)));
    drivercontroller.povDown().onTrue(new ArmHoming(m_arm));
    operatorController.cross().onTrue(new ArmWithVision(m_arm, m_vision));
  }

  public void shooterBinding(){
    drivercontroller.triangle().onTrue(new InstantCommand(() -> m_shooter.setMotionMagicVelocity(6000)));
    drivercontroller.cross().onTrue(new InstantCommand(() -> m_shooter.disableMotors()));
    operatorController.cross().onTrue(new ShooterWithVision(m_shooter, m_vision));
  }


  public void intakeBinding(){
    drivercontroller.R2().onTrue(new InstantCommand(() -> m_intakeSub.setMotorPrecent(-0.4)));
    drivercontroller.L2().onTrue(new InstantCommand(() -> m_intakeSub.setMotorPrecent(0.4)));
    operatorController.L2().onTrue(new IntakeEatNote(m_intakeSub));
    }

  public void presets(){
    drivercontroller.R3().onTrue(new Amping(m_arm, m_shooter, m_intakeSub, drivercontroller.R3()));
    driverStationController.LeftBlue().onTrue(new Preset(m_shooter, m_arm, 20, 6000));
    driverStationController.RightYellow().onTrue(new Preset(m_shooter, m_arm, 40, 8000));
    driverStationController.DownYellow().onTrue(new Preset(m_shooter, m_arm, 60, 7000));
    driverStationController.UpBlue().onTrue(new Preset(m_shooter, m_arm, 70, 5000));
  }

  public void resets(){
    operatorController.L1().onTrue(new Preset(m_shooter, m_arm, 9.57, 0));
  }
}