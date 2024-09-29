
package frc.robot.commands.ShooterCommands;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.DELib.Subsystems.Vision.VisionSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterWithVision extends Command {

  private ShooterSubsystem m_ShooterSubsystem;
  private VisionSubsystem m_VisionSubsystem;
  private LinearFilter m_filter;
  private double Lastsetpoint = -9999;
  private double threshold = 0.05;

  public ShooterWithVision(ShooterSubsystem shooterSubsystem,VisionSubsystem visionSubsystem) {
    m_ShooterSubsystem = shooterSubsystem;
    m_VisionSubsystem = visionSubsystem;
    m_filter = LinearFilter.movingAverage(5);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_VisionSubsystem.getTv()){
      if(Math.abs(Lastsetpoint - m_VisionSubsystem.getTy()) > threshold){
        m_ShooterSubsystem.setUsingInterpulation(m_filter.calculate(Lastsetpoint), true);
        Lastsetpoint = m_VisionSubsystem.getTy();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
