// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.DELib.BooleanUtil.StableBoolean;
import frc.robot.subsystems.ArmSubsystem;

public class ArmHoming extends Command {
  ArmSubsystem m_arm;
  StableBoolean isAtResetPoint;
  boolean done = false;
  boolean skipSetPosition = false;
  /** Creates a new ArmHoming. */
  public ArmHoming(ArmSubsystem arm ) {
    m_arm = arm;
    isAtResetPoint = new StableBoolean(0.2);
    addRequirements(arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.ControlSoftLimit(false);
    done = false;
    if(m_arm.getPosition()<20){
    skipSetPosition = true;
    }
    if(!skipSetPosition){
      m_arm.setMotionMagicPosition(20);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_arm.isAtSetpoint() || skipSetPosition){
      m_arm.setPrecentOutput(-0.07);
    }
    if (isAtResetPoint.get(m_arm.getMagnetState())) {
      m_arm.resetSubsystemToInitialState();
      done = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.setMotionMagicPosition(20);
    skipSetPosition = false;
    done = false;
    m_arm.ControlSoftLimit(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
