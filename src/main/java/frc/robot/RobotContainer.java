// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.proto.Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.DELib.Subsystems.Swerve.SwerveSubsystem;
import frc.DELib.Subsystems.Swerve.SwerveCommands.ResetSwerveModules;
import frc.DELib.Subsystems.Swerve.SwerveCommands.TeleopDrive;
import frc.DELib.Util.SwerveAutoBuilder;
import frc.robot.commands.IntakeCommnands.IntakeEatNote;
import frc.robot.subsystems.IntakeSubsystem;

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
  private SwerveAutoBuilder swerveAutoBuilder;
  public RobotContainer() {
    m_swerve = SwerveSubsystem.getInstance(Constants.Swerve.swerveConstants);
    swerveAutoBuilder = new SwerveAutoBuilder(m_swerve);
    m_swerve.setDefaultCommand(new TeleopDrive(m_swerve, controller, controller.L1(), controller.touchpad(),
    controller.options(), () -> false));
    SmartDashboard.putData("calibrate Swerve Modules", new ResetSwerveModules(m_swerve).ignoringDisable(true));
    SmartDashboard.putData("reset Odometry",
    new InstantCommand(() -> m_swerve.resetOdometry(new Pose2d())).ignoringDisable(true));
    m_intakeSub = IntakeSubsystem.getInstance();
  }

  public void disableMotors() {
    m_swerve.disableModules();
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

  public void intakeBinding(){
    controller.L2().onTrue(new IntakeEatNote(m_intakeSub));
  }
}