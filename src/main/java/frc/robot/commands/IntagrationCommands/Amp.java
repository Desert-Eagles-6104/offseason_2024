// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntagrationCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.DELib.Subsystems.ServoSubsystem.Commands.ServoSubsystemSetPosition;
import frc.robot.Constants.Shooter;
import frc.robot.commands.ArmCommands.ArmSetPosition;
import frc.robot.commands.IntakeCommnands.IntakeSetPrecent;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Amp extends SequentialCommandGroup {
  /** Creates a new Amp. */

  public Amp(IntakeSubsystem intake, ArmSubsystem arm, ShooterSubsystem shooter) {
    addCommands(new SmartPreset(shooter, arm, 95, 1000));
  }
}
