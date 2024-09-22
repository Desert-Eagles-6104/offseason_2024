// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.DELib.Sysid;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Consumer;
import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog.State;

/** Add your docs here. */
public class SysidConfiguration {
    /** The voltage ramp rate used for quasistatic test routines. */
    public Measure<Velocity<Voltage>> m_rampRate = Volts.of(1).per(Seconds.of(1));

    /** The step voltage output used for dynamic test routines. */
    public Measure<Voltage> m_stepVoltage = Volts.of(7);

    /** Safety timeout for the test routine commands. */
    public Measure<Time> m_timeout = Seconds.of(10);

    /** Optional handle for recording test state in a third-party logging solution. */
    public final Consumer<State> m_recordState = (state) -> SignalLogger.writeString("state", state.toString());

    /**
     * Returns measured data (voltages, positions, velocities) of the mechanism motors during test
     * routines.
     */
    public final Consumer<SysIdRoutineLog> m_log = null;
    //#endregion mechanisem
}