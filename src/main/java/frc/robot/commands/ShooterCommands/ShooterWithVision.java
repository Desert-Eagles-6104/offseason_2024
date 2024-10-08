
package frc.robot.commands.ShooterCommands;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.DELib.Subsystems.Vision.VisionSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterWithVision extends Command {

  private ShooterSubsystem m_Shooter;
  private LinearFilter m_filter;
  private double Lastsetpoint = -9999;
  private double threshold = 0.05;

  public ShooterWithVision(ShooterSubsystem shooter) {
    m_Shooter = shooter;
    m_filter = LinearFilter.movingAverage(5);
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(VisionSubsystem.getTv()){
      if(Math.abs(Lastsetpoint - VisionSubsystem.getTy()) > threshold){
        m_Shooter.setUsingInterpulation(m_filter.calculate(Lastsetpoint), false);
        Lastsetpoint = VisionSubsystem.getTy();
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
