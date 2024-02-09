package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IOConstants;
import wildlib.NotYetImplemented;
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
    }

    public void initDefaultCommand() {
        setDefaultCommand(Commands.run(() -> {
            throw new NotYetImplemented();
        }));
    }

    public Command advance() {
        throw new NotYetImplemented();
    }

    public void stop() {
        throw new NotYetImplemented();
    }
    
    /**
     * Sets the speed of the intake wheels
     * 
     * @param speed The speed value to set. Should be between -1.0 and 1.0.
     */
    public void setSpeed(double speed) {
        throw new NotYetImplemented();
    }

    public boolean noteDetected() {
        throw new NotYetImplemented();
    }

    public Command waitForNote() {
        throw new NotYetImplemented();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Rotations", m_drive.getPosition());
        SmartDashboard.putBoolean("Note Ready", noteDetected());
    }
}
