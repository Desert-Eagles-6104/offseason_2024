
package frc.robot.commands.ShooterCommands;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.DELib.Subsystems.Vision.VisionSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterWithVision extends Command {

  private ShooterSubsystem m_Shooter;
  private VisionSubsystem m_Vision;
  private LinearFilter m_filter;
  private double Lastsetpoint = -9999;
  private double threshold = 0.05;

  public ShooterWithVision(ShooterSubsystem shooter,VisionSubsystem visionSubsystem) {
    m_Shooter = shooter;
    m_Vision = visionSubsystem;
    m_filter = LinearFilter.movingAverage(5);
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_Vision.getTv()){
      if(Math.abs(Lastsetpoint - m_Vision.getTy()) > threshold){
        m_Shooter.setUsingInterpulation(m_filter.calculate(Lastsetpoint), true);
        Lastsetpoint = m_Vision.getTy();
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
