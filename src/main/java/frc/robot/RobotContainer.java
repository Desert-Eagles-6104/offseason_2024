// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.DELib.Subsystems.ServoSubsystem.Base.Motor.ServoSubsystemTalon;
import frc.DELib.Subsystems.Swerve.SwerveSubsystem;
import frc.DELib.Subsystems.Swerve.SwerveCommands.RotateToTarget;
import frc.DELib.Subsystems.Swerve.SwerveCommands.TeleopDrive;
import frc.DELib.Subsystems.Vision.VisionSubsystem;
import frc.DELib.Subsystems.Vision.VisionUtil.CameraSettings;
import frc.robot.commands.ArmCommands.ArmChangeNeutralMode;
import frc.robot.commands.ArmCommands.ArmHoming;
import frc.robot.commands.IntagrationCommands.Preset;
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
  private CommandPS5Controller controller = new CommandPS5Controller(0);
  private SwerveSubsystem m_swerve;
  private IntakeSubsystem m_intakeSub;
  private ArmSubsystem m_arm;
  private ShooterSubsystem m_shooter;
  private VisionSubsystem m_vision;
  // private PhoneixSysid sysid;
  // private SwerveAutoBuilder swerveAutoBuilder;
  public RobotContainer() {
    m_swerve = SwerveSubsystem.createInstance(Constants.Swerve.swerveConstants);
    m_intakeSub = IntakeSubsystem.getInstance();
    m_arm = new ArmSubsystem(Constants.arm.configuration);
    m_shooter = new ShooterSubsystem(Constants.Shooter.configuration);
    m_vision = new VisionSubsystem(new CameraSettings(0, 0, 0, 0, 0, 0, true), new CameraSettings(0, 0, 0, 0, 0, 0, false));
    // swerveAutoBuilder = new SwerveAutoBuilder(m_swerve);
    // SmartDashboard.putData("calibrate Swerve Modules", new ResetSwerveModules(m_swerve).ignoringDisable(true));
    // SmartDashboard.putData("reset Odometry",
    // new InstantCommand(() -> m_swerve.resetOdometry(new Pose2d())).ignoringDisable(true));
    m_swerve.setDefaultCommand(new TeleopDrive(m_swerve, m_vision, controller, controller.L1(), controller.touchpad(), controller.options(), controller.R1()));
    // controller.L1().onTrue(new RotateToTarget(m_swerve, m_vision, controller));
    // sysid = new PhoneixSysid(Constants.sysidConfiguration, m_shooter);
    // controller.circle().onTrue(sysid.runFullCharacterization(true));
    
    //arm
    SmartDashboard.putData("Change Arm  NeutralMode", new ArmChangeNeutralMode(m_arm).ignoringDisable(true));
    SmartDashboard.putData("reset arm", new InstantCommand(() -> m_arm.resetPosition(9.57)).ignoringDisable(true));
    controller.circle().onTrue(new InstantCommand(() -> m_arm.setMotionMagicPosition(90)));
    controller.square().onTrue(new InstantCommand(() -> m_arm.setMotionMagicPosition(45)));
    controller.PS().onTrue(new InstantCommand(() -> m_arm.setMotionMagicPosition(Constants.arm.configuration.homePosition)));
    controller.povDown().onTrue(new ArmHoming(m_arm));
    // shooter
    controller.triangle().onTrue(new InstantCommand(() -> m_shooter.setMotionMagicVelocity(6000)));
    controller.cross().onTrue(new InstantCommand(() -> m_shooter.disableMotors()));
    //intake
    controller.R2().onTrue(new InstantCommand(() -> m_intakeSub.setMotorPrecent(-0.4)));
    controller.L2().onTrue(new InstantCommand(() -> m_intakeSub.setMotorPrecent(0.4)));

    SmartDashboard.putData("presetCheak", new Preset(m_shooter, m_arm, 20, 6000));
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
    return null;
  }

  public void intakeBinding(){
    // controller.L2().onTrue(new IntakeEatNote(m_intakeSub));
    }
}