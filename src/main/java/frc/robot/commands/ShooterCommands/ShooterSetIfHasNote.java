// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterSetIfHasNote extends Command {
  /** Creates a new ShooterSetIfNote. */
  private ShooterSubsystem m_shooter;
  private IntakeSubsystem m_intake;
  private double m_velocity;

  public ShooterSetIfHasNote(ShooterSubsystem shooter, IntakeSubsystem intake, double velocity) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_velocity = velocity;
    m_intake = intake;
    m_shooter = shooter;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.setMotionMagicVelocityWithRatio(0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.setMotionMagicVelocityWithRatio(m_velocity);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_intake.hasGamePiece();
  }
}
