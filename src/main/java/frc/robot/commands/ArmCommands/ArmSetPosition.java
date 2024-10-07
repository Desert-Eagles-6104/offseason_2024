// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands;

import frc.DELib.Subsystems.ServoSubsystem.Base.Motor.ServoSubsystemTalon;
import frc.DELib.Subsystems.ServoSubsystem.Commands.ServoSubsystemSetPosition;

/** Add your docs here. */
public class ArmSetPosition extends ServoSubsystemSetPosition {

    public ArmSetPosition(ServoSubsystemTalon ServoSubsystemTalon, double Position, boolean motionMagic) {
        super(ServoSubsystemTalon, Position, motionMagic);
    }}