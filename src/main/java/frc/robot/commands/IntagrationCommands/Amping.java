// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntagrationCommands;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class Amping extends Command {
  ArmSubsystem m_arm;
  ShooterSubsystem m_shooter;
  IntakeSubsystem m_intake;
  BooleanSupplier m_start;
  double m_startPos = 70.0;
  double m_shootingPosition = 95.0;
  double m_startTriggerPosition = 60.0;
  double m_Velocity = 300;
  double m_intakePrecent = 0.3;
  /** Creates a new Amping. */
  public Amping(ArmSubsystem arm , ShooterSubsystem shooter , IntakeSubsystem intake, BooleanSupplier start) {
    m_arm = arm;
    m_shooter = shooter;
    m_intake = intake;
    m_start = start;
    addRequirements(arm, shooter, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.setMotionMagicPosition(m_startPos);
    m_shooter.setMotionMagicVelocity(m_Velocity);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Math.abs(m_arm.getPosition() - m_startPos) < 2 && !m_start.getAsBoolean()){
      m_arm.setMotionMagicPosition(m_shootingPosition);
    }

    if(m_arm.getPosition() > m_startTriggerPosition){
      m_intake.setMotorPrecent(m_intakePrecent);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_start = () -> false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_arm.getPosition() - m_shootingPosition) < 2;
  }
}
