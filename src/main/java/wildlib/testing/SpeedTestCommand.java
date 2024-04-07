package wildlib.testing;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Swerve;

public class SpeedTestCommand extends Command{
    private final Swerve m_drive;

    private double m_maxAngularSpeed = 0;
    private double m_maxAngularAcceleration = 0;
    private double m_maxTranslationalSpeed = 0;
    private double m_maxTranslationalAcceleration = 0;
    private double m_previousNanos = 0;
    private double m_previousAngleV = 0;
    private double m_angleA = 0;

    public SpeedTestCommand (Swerve drive){
        m_drive = drive;
    }

    @Override
    public void initialize (){
        m_previousNanos = System.nanoTime();
        m_previousAngleV = Math.toRadians(m_drive.getAngularVelocity());
    }

    @Override
    public void execute(){
        double transV = m_drive.getTranslationalVelocity();
        double transA = m_drive.getTranslationAcceleration();
        double angleV = Math.toRadians(m_drive.getAngularVelocity());
        if (System.nanoTime() - m_previousNanos == 5e6) {
            m_angleA = (angleV - m_previousAngleV)/((System.nanoTime() - m_previousNanos)/1e9);
        }
        SmartDashboard.putNumber("Translational Speed", transV);
        SmartDashboard.putNumber("Translational Acceleration (ABS)", transA);
        SmartDashboard.putNumber("Angular Velocity", angleV);
        SmartDashboard.putNumber("Angular Acceleration", m_angleA);
        m_maxTranslationalSpeed = (transV > m_maxTranslationalSpeed) ? transV : m_maxTranslationalSpeed;
        m_maxTranslationalAcceleration = (transA > m_maxTranslationalAcceleration) ? transA : m_maxTranslationalAcceleration;
        m_maxAngularSpeed = (Math.abs(angleV) > m_maxAngularSpeed) ? Math.abs(angleV) : m_maxAngularSpeed;
        m_maxAngularAcceleration = (Math.abs(m_angleA) > m_maxAngularAcceleration) ? Math.abs(m_angleA) : m_maxAngularAcceleration;

        System.out.printf(
        "Max Translational Speed: %f%n Max Translational Acceleration (ABS): %f%n Max Angular Velocity: %f%n Max Angular Acceleration",
         m_maxTranslationalSpeed,
         m_maxTranslationalAcceleration, 
         m_maxAngularSpeed, 
         m_maxAngularAcceleration
        );
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}