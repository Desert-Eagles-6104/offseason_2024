package frc.DELib.Subsystems.Swerve.SwerveUtil;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.DELib.Subsystems.Vision.VisionSubsystem;

public class DriveAssist {    
    private double m_kp;
    private BooleanSupplier m_intakeButton;
    private LinearFilter m_filter;

    public DriveAssist (double kp, BooleanSupplier intakeButton){
        m_kp = kp;
        m_intakeButton = intakeButton;
        m_filter = LinearFilter.movingAverage(4);
    }

    // public ChassisSpeeds update(ChassisSpeeds chassisSpeeds, Rotation2d robotHeading){
    //     ChassisSpeeds toReturn;
    //     if(VisionSubsystem.getTvNote() && m_intakeButton.getAsBoolean()){
    //         toReturn = ChassisSpeeds.fromRobotRelativeSpeeds(0, m_filter.calculate(VisionSubsystem.getTxNote())*m_kp, 0, robotHeading);
    //     }
    //     chassisSpeeds.vxMetersPerSecond = toReturn.vxMetersPerSecond + chassisSpeeds.vxMetersPerSecond;
    //     chassisSpeeds.vyMetersPerSecond = toReturn.vyMetersPerSecond + chassisSpeeds.vyMetersPerSecond;
    //     return chassisSpeeds;
    // }
}
