package frc.DELib.Subsystems.Swerve.SwerveUtil;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import frc.DELib.BooleanUtil.StickyBoolean;
import frc.robot.Constants;

public class HeadingController  {

    private PIDController m_pidControllerStabalize;
    private PIDController m_pidControllerSnap;
    private PIDController m_pidControllerVision;

    private Rotation2d m_lastHeading;
    private boolean m_shouldSaveHeading = true;
    private StickyBoolean m_useVisionLatch;
    private boolean firstRun = true;

   /**
    *  maintains the set direction by the joystick within a 3 degree error of the joystick
    * @param swerve sweve subsystem object
    * @param kp p value
    * @param ki i value
    * @param kd d value
    */
    public HeadingController (double kp , double ki , double kd){
        m_pidControllerStabalize = new PIDController(kp, ki, kd);
        m_pidControllerStabalize.enableContinuousInput(-180, 180);
        m_pidControllerStabalize.setIntegratorRange(-1*(9.9*Math.E+30), (9.9*Math.E+30));
        m_useVisionLatch = new StickyBoolean();
    }
    /**
     * holds the current heading of the swerve and sets it to the pid controller
     * @param currentHeading current heading of the robot
     * @return a setpoint to the pid controller
     */
    public double update(Rotation2d currentHeading){
        return -clamp(m_pidControllerStabalize.calculate(currentHeading.getDegrees()),-Constants.Swerve.swerveConstants.maxAngularVelocity,Constants.Swerve.swerveConstants.maxAngularVelocity);
    }
    /**
     * calculates the heading of the robot
     * @param shouldRun a boolean to decide whether or not to use the headingController function
     * @param chassisSpeeds x,y and omega speeds 
     * @return new values for the modules to use 
     */
    public ChassisSpeeds calculateOmegaSpeed(boolean shouldRun ,boolean isSwerveReset, boolean useVision ,ChassisSpeeds chassisSpeeds, Rotation2d robotHeading, Rotation2d visionHeading){
      m_useVisionLatch.update(useVision);
      if(shouldRun && !isSwerveReset){
        if(firstRun){
          setSetpoint(robotHeading);
          firstRun = false;
        }
        if(!m_useVisionLatch.get()){
          if (Math.abs(chassisSpeeds.omegaRadiansPerSecond) < 0.05){ // was 0.15
            if(m_shouldSaveHeading){
              m_lastHeading = robotHeading;
              setSetpoint(m_lastHeading);
              m_shouldSaveHeading = false;
            }
            chassisSpeeds.omegaRadiansPerSecond = update(robotHeading);
            return chassisSpeeds;
          }
          else{
            m_shouldSaveHeading= true;
          }
        }
        else if(m_useVisionLatch.get()){
          if (Math.abs(chassisSpeeds.omegaRadiansPerSecond) > 0.05){
            m_useVisionLatch.reset();
          }
          chassisSpeeds.omegaRadiansPerSecond = update(visionHeading);
          return chassisSpeeds;
        }
      }
      else if(isSwerveReset){
        setSetpoint(DriverStation.getAlliance().isPresent() && (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) ? Rotation2d.fromDegrees(180):Rotation2d.fromDegrees(0)); 
      }
      return chassisSpeeds;
  }
    /**
     * sets a setpoint for the pid controller
     * @param setpoint
     */
    public void setSetpoint(Rotation2d setpoint){
        if(setpoint.getDegrees() == m_pidControllerStabalize.getSetpoint()) return;
        m_pidControllerStabalize.reset();
        m_pidControllerStabalize.setSetpoint(setpoint.getDegrees());
      }

    /**
   @param value clamped value
  @param min min value
  @param max max value
  @return sets a range for the value if its between the max and min points
  */
  private double clamp(double value, double min, double max) {
    return Math.max(min, Math.min(max, value));
  }
}
