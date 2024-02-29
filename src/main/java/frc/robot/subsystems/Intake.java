package frc.robot.subsystems;

import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
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
    private final PIDSpark m_motor;
    private final AnalogInput m_detector;

    private static Intake m_instance;

    public static Intake getInstance() {
        if (m_instance == null) {
            m_instance = new Intake(
                new PIDSpark(
                    IOConstants.intakeId,
                    MotorType.kBrushless,
                    IntakeConstants.motorKP,
                    IntakeConstants.motorKI,
                    IntakeConstants.motorKD,
                    IntakeConstants.motorKF
                ),
                IOConstants.detectorChannel
            );
        }

        return m_instance;
    }

    private Intake(PIDSpark drive, int detectorChannel) {
        m_motor = drive;
        m_detector = new AnalogInput(detectorChannel);

        m_motor.setPositionConversionFactor(IntakeConstants.distanceFactor);
        m_motor.setIdleMode(IdleMode.kBrake);
        m_motor.setInverted(true);
    }

    public void initDefaultCommand() {
        Command defaultCommand = Commands.either(
            Commands.runOnce(m_motor::stopMotor),
            Commands.runOnce(() -> m_motor.set(-0.3)),
            this::noteDetected
        );
        defaultCommand.addRequirements(this);

        setDefaultCommand(defaultCommand);    
    }

    public REVLibError setTargetVelocity(double velocity) {
        return m_motor.setTargetVelocity(velocity);
    }

    public Command advanceAmp() {
        Command advance = waitForNote(0.5)
            .andThen(
                Commands.either(Commands.sequence(
                    Commands.runOnce(() -> m_motor.setTargetVelocity(IntakeConstants.ampTarget)),
                    // TODO: Tune the wait time
                    Commands.waitSeconds(1.0),
                    Commands.runOnce(m_motor::stopMotor)
                ), Commands.none(), this::noteDetected)
            );
        advance.addRequirements(this);
        
        return advance;
    }

    public Command advanceSpeaker() {        
        Command advance = waitForNote(0.5)
            .andThen(
                Commands.either(Commands.sequence(
                    Commands.runOnce(() -> m_motor.setTargetVelocity(IntakeConstants.speakerTarget)),
                    // TODO: Tune the wait time
                    Commands.waitSeconds(1.0),
                    Commands.runOnce(m_motor::stopMotor)
                ), Commands.none(), this::noteDetected)
            );
        advance.addRequirements(this);
        
        return advance;
    }

    public void stop() {
        m_motor.stopMotor();
    }
    
    /**
     * Sets the speed of the intake wheels
     * 
     * @param speed The speed value to set. Should be between -1.0 and 1.0.
     */
    public void setSpeed(double speed) {
        m_motor.set(speed);
    }

    public boolean noteDetected() {
        return m_detector.getVoltage() < 3.0;
    }

    /**
     * Waits for a Note to be detected in the conveyor.
     * 
     * @param timeout How long to wait before giving up.
     * @return A command that waits for a Note
     */
    public Command waitForNote(double timeout) {
        if (noteDetected()) {
            return Commands.none();
        } else {
            Command waitFor = Commands.runOnce(() -> m_motor.setTargetVelocity(IntakeConstants.idleTarget))
                .andThen(
                    Commands.waitUntil(this::noteDetected).withTimeout(timeout),
                    Commands.runOnce(m_motor::stopMotor)
                );
            waitFor.addRequirements(this);
            return waitFor;
        }
    }

    public double getVelocity() {
        return m_motor.getVelocity();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Velocity", m_motor.getVelocity());
        SmartDashboard.putBoolean("Note Ready", noteDetected());
        SmartDashboard.putNumber("Detector Voltage", m_detector.getVoltage());

        m_motor.getPIDController().setP(SmartDashboard.getNumber("Intake P", IntakeConstants.motorKP));
    }
}
