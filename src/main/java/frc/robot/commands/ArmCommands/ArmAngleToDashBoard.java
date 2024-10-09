// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ArmAngleToDashBoard extends Command {
  ArmSubsystem m_armSubsystem;
  ShooterSubsystem m_shooter;
  public ArmAngleToDashBoard(ArmSubsystem armSubsystem, ShooterSubsystem shooter) {
    m_armSubsystem = armSubsystem;
    m_shooter = shooter;
    addRequirements(armSubsystem);
  }

  @Override
  public void initialize() {
    m_shooter.setVelocityWithRatio(7000);
  }

  @Override
  public void execute() {
    m_armSubsystem.Print();
  }

  @Override
  public void end(boolean interrupted) {
    
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
