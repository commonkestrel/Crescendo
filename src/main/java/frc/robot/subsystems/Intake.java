package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IOConstants;
import wildlib.PIDSpark;

/** 
 * Subsystem representing the ground intake on our robot
 * 
 * @author Jett Bergthold
 */
public class Intake extends SubsystemBase {
    private final PIDSpark m_drive;
    private final DigitalInput m_detector;

    private static Intake m_instance;

    public static Intake getInstance() {
        if (m_instance == null) {
            m_instance = new Intake(
                new PIDSpark(
                    IOConstants.intakeId,
                    MotorType.kBrushless,
                    IntakeConstants.driveKP,
                    IntakeConstants.driveKI,
                    IntakeConstants.driveKD
                ),
                IOConstants.detectorChannel
            );
        }

        return m_instance;
    }

    private Intake(PIDSpark drive, int detectorChannel) {
        m_drive = drive;
        m_detector = new DigitalInput(detectorChannel);

        m_drive.setPositionConversionFactor(IntakeConstants.distanceFactor);
        m_drive.setIdleMode(IdleMode.kBrake);
    }

    public void initDefaultCommand() {
        setDefaultCommand(Commands.either(
            Commands.run(m_drive::stopMotor),
            Commands.run(() -> m_drive.setTargetVelocity(IntakeConstants.idleTarget)),
            m_detector::get
        ));    
    }

    public Command advanceAmp() {
        Command advance = waitForNote(0.5)
            .andThen(
                Commands.either(Commands.sequence(
                    Commands.runOnce(() -> m_drive.setTargetVelocity(IntakeConstants.ampTarget)),
                    // TODO: Tune the wait time
                    Commands.waitSeconds(1.0),
                    Commands.runOnce(m_drive::stopMotor)
                ), Commands.none(), this::noteDetected)
            );
        advance.addRequirements(this);
        
        return advance;
    }

    public Command advanceSpeaker() {        
        Command advance = waitForNote(0.5)
            .andThen(
                Commands.either(Commands.sequence(
                    Commands.runOnce(() -> m_drive.setTargetVelocity(IntakeConstants.speakerTarget)),
                    // TODO: Tune the wait time
                    Commands.waitSeconds(1.0),
                    Commands.runOnce(m_drive::stopMotor)
                ), Commands.none(), this::noteDetected)
            );
        advance.addRequirements(this);
        
        return advance;
    }

    public void stop() {
        m_drive.stopMotor();
    }
    
    /**
     * Sets the speed of the intake wheels
     * 
     * @param speed The speed value to set. Should be between -1.0 and 1.0.
     */
    public void setSpeed(double speed) {
        m_drive.set(speed);
    }

    public boolean noteDetected() {
        return m_detector.get();
    }

    /**
     * Waits for a Note to be detected in the conveyor.
     * 
     * @param timeout How long to wait before giving up.
     * @return A command that waits for a Note
     */
    public Command waitForNote(double timeout) {
        if (m_detector.get()) {
            return Commands.none();
        } else {
            Command waitFor = Commands.runOnce(() -> m_drive.setTargetVelocity(IntakeConstants.idleTarget))
                .andThen(
                    Commands.waitUntil(m_detector::get).withTimeout(timeout),
                    Commands.runOnce(m_drive::stopMotor)
                );
            waitFor.addRequirements(this);
            return waitFor;
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Velocity", m_drive.getVelocity());
        SmartDashboard.putBoolean("Note Ready", noteDetected());
    }
}
