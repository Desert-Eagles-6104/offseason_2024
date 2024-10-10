// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntagrationCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ArmCommands.ArmHoming;
import frc.robot.commands.IntakeCommnands.IntakeForTime;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Amp extends SequentialCommandGroup {
  /** Creates a new Amp. */
  // 103
  public Amp(IntakeSubsystem intake, ArmSubsystem arm, ShooterSubsystem shooter) {
    addCommands(new InstantCommand(() -> arm.ControlSoftLimit(false)),new SmartPreset(shooter, arm, 70, 750, false),new WaitCommand(0), new IntakeForTime(intake, 0.6, 0.3).alongWith(new SmartPreset(shooter, arm, 100, 750, false)), new WaitCommand(0.5), new SmartPreset(shooter, arm, 20, 0), new WaitCommand(0.1),new ArmHoming(arm) ,new InstantCommand(() -> arm.ControlSoftLimit(true)));
  }
}
