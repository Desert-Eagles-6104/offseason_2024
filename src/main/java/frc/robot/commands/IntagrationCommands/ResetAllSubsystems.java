// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntagrationCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.ArmCommands.ArmHoming;
import frc.robot.commands.IntakeCommnands.IntakeSetPrecent;
import frc.robot.commands.ShooterCommands.ShooterSetVelocity;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ResetAllSubsystems extends ParallelCommandGroup {
  /** Creates a new ResetAllSubsystems. */
  public ResetAllSubsystems(ShooterSubsystem shooter, IntakeSubsystem intake, ArmSubsystem armSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ArmHoming(armSubsystem), new ShooterSetVelocity(shooter, 0), new InstantCommand(() -> intake.setMotorPrecent(0), intake));
  }
}
