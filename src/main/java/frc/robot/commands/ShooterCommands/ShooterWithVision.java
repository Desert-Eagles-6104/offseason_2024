
package frc.robot.commands.ShooterCommands;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.DELib.Subsystems.Vision.VisionSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterWithVision extends Command {

  ShooterSubsystem m_ShooterSubsystem;
  VisionSubsystem m_VisionSubsystem;
  MedianFilter m_filter;  

  public ShooterWithVision(ShooterSubsystem shooterSubsystem,VisionSubsystem visionSubsystem) {
    m_ShooterSubsystem = shooterSubsystem;
    m_VisionSubsystem = visionSubsystem;
    m_filter = new MedianFilter(5);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_VisionSubsystem.getTv()){
      m_ShooterSubsystem.setUsingInterpulation(m_VisionSubsystem.getTy(), true);
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
