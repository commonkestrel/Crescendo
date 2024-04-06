package frc.robot.subsystems;

import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import wildlib.io.PIDSpark;
import frc.robot.Constants.IOConstants;

/** 
 * Subsystem representing the ground intake on our robot
 * 
 * @author Jett Bergthold
 */
public class Intake extends SubsystemBase {
    private final PIDSpark m_indexer;
    private final PIDSpark m_preroller;
    private final DigitalInput m_detector;
    private final DigitalInput m_noteBeambreak;
    private boolean m_overrideSensor = false;

    private static Intake m_instance;

    public static Intake getInstance() {
        if (m_instance == null) {
            m_instance = new Intake(
                new PIDSpark(
                    IOConstants.indexerId,
                    MotorType.kBrushless,
                    PIDSpark.SparkFlexModel(),
                    IntakeConstants.indexerKP,
                    IntakeConstants.indexerKI,
                    IntakeConstants.indexerKD,
                    IntakeConstants.indexerKF
                ),
                new PIDSpark(
                    IOConstants.prerollerId,
                    MotorType.kBrushless,
                    PIDSpark.SparkMaxModel(),
                    IntakeConstants.prerollerKP,
                    IntakeConstants.prerollerKI,
                    IntakeConstants.prerollerKD,
                    IntakeConstants.prerollerKF
                ),
                IOConstants.detectorChannel,
                IOConstants.noteBeambreakChannel
            );
        }

        return m_instance;
    }

    private Intake(PIDSpark indexer, PIDSpark preroller, int detectorChannel, int noteBeambreakChannel) {
        m_indexer = indexer;
        m_preroller = preroller;
        m_detector = new DigitalInput(detectorChannel);
        m_noteBeambreak = new DigitalInput(noteBeambreakChannel);

        m_indexer.setPositionConversionFactor(IntakeConstants.distanceFactor);
        m_indexer.setIdleMode(IdleMode.kBrake);
        m_indexer.setInverted(true);
        m_indexer.setSmartCurrentLimit(100);
        m_indexer.burnFlash();

        m_preroller.setIdleMode(IdleMode.kBrake);
        m_preroller.setSmartCurrentLimit(40);
        m_preroller.burnFlash();
    }

    /**
     * Sets the target velocity for the indexer wheels.
     * 
     * @param velocity The reference velocity to use in the velocity PID (in RPM)
     * @return {@link REVLibError.kOk} if successful.
     */
    public REVLibError setTargetIndexerVelocity(double velocity) {
        return m_indexer.setTargetVelocity(velocity);
    }

    /**
     * Sets the target velocity for the preroller wheels.
     * 
     * @param velocity The reference velocity to use in the velocity PID (in RPM)
     * @return {@link REVLibError.kOk} if successful.
     */
    public REVLibError setTargetPrerollerVelocity(double velocity) {
        return m_preroller.setTargetVelocity(velocity);
    }

    public void stop() {
        m_indexer.stopMotor();
        m_preroller.stopMotor();
    }
    
    /**
     * Sets the speed of the indexer wheels
     * 
     * @param speed The speed value to set. Should be between -1.0 and 1.0.
     */
    public void setIndexerSpeed(double speed) {
        m_indexer.set(speed);
    }

    /**
     * Sets the speed of the preroller wheels
     * 
     * @param speed The speed value to set. Should be between -1.0 and 1.0.
     */
    public void setPrerollerSpeed(double speed) {
        m_preroller.set(speed);
    }

    public boolean noteDetected() {
        return m_overrideSensor || !m_noteBeambreak.get();

    }

    public double getIndexerVelocity() {
        return m_indexer.getVelocity();
    }

    public double getPrerollerVelocity() {
        return m_preroller.getVelocity();
    }

    public void toggleOverride() {
        m_overrideSensor = !m_overrideSensor;
    }

    public boolean getOverride() {
        return m_overrideSensor;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Current", m_indexer.getOutputCurrent());
        SmartDashboard.putNumber("Indexer Velocity", m_indexer.getVelocity());
        SmartDashboard.putNumber("Preroller Velocity", m_preroller.getVelocity());
        SmartDashboard.putBoolean("Note Ready", noteDetected());
        SmartDashboard.putBoolean("Beam Broken", m_noteBeambreak.get());
        SmartDashboard.putBoolean("Note Limit Switch Closed", m_detector.get());
        // SmartDashboard.putNumber("Detector Voltage", m_detector.getVoltage());
    }
}
