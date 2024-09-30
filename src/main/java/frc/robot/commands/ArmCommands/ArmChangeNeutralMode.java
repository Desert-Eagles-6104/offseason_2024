// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ArmSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmChangeNeutralMode extends InstantCommand {
  private ArmSubsystem m_arm;
  private boolean hasChanged = false;
  public ArmChangeNeutralMode(ArmSubsystem arm) {
    m_arm = arm;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(!hasChanged){
      m_arm.changeNeutralMode(NeutralModeValue.Coast);
      hasChanged = true;
    }
    else if(hasChanged){
      m_arm.changeNeutralMode(NeutralModeValue.Brake);
      hasChanged = false;
    }
  }
}
