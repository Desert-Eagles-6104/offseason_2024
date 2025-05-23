// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.DELib.Subsystems.VelocitySubsystem.Commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.DELib.Subsystems.VelocitySubsystem.Base.Motor.VelocitySubsystemTalon;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class VelocitySubsystemDisableMotors extends InstantCommand {
  private VelocitySubsystemTalon m_velocitySubsystemTalon;

  public VelocitySubsystemDisableMotors(VelocitySubsystemTalon velocitySubsystemTalon) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(velocitySubsystemTalon);
    m_velocitySubsystemTalon = velocitySubsystemTalon;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_velocitySubsystemTalon.disableMotors();
  }
}
