// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntagrationCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoShoot extends Command {
  /** Creates a new AutoShoot. */
  private ShooterSubsystem m_shooter;
  private ArmSubsystem m_arm;
  private IntakeSubsystem m_intake;
  private boolean isFinish = false;
  public AutoShoot(ShooterSubsystem shooter, ArmSubsystem arm, IntakeSubsystem intake) {
    m_shooter = shooter;
    m_intake = intake;
    m_arm = arm;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinish = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_arm.isAtSetpoint() && m_shooter.isAtSetpoint() && m_shooter.getRightSetpoint() == 7000){
      m_intake.setMotorPrecent(0.3);
      isFinish = !m_intake.secondBeamBreak();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinish;
  }
}
