package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConveyorConstants;
import frc.robot.Constants.IOConstants;
import wildlib.PIDSpark;

public class Conveyor extends SubsystemBase {
    private final PIDSpark m_drive;
    private final DigitalInput m_detector;

    private static Conveyor m_instance;

    public static Conveyor getInstance() {
        if (m_instance == null) {
            m_instance = new Conveyor(
                new PIDSpark(
                    IOConstants.conveyorId,
                    MotorType.kBrushless,
                    ConveyorConstants.driveKP,
                    ConveyorConstants.driveKI,
                    ConveyorConstants.driveKD
                ),
                ConveyorConstants.detectorChannel
            );
        }

        return m_instance;
    }

    private Conveyor(PIDSpark drive, int detectorChannel) {
        m_drive = drive;
        m_detector = new DigitalInput(detectorChannel);

        m_drive.setPositionConversionFactor(ConveyorConstants.distanceFactor);
    }

    public Command advance() {
        Command wait = Commands
            .runOnce(() -> m_drive.set(0.8))
            .andThen(Commands.waitUntil(() -> !noteDetected()));
        wait.addRequirements(this);

        return wait;
    }

    public void stop() {
        m_drive.set(0);
    }

    public void setSpeed(double speed) {
        m_drive.set(speed);
    }

    public boolean noteDetected() {
        return !m_detector.get();
    }

    public Command waitForNote() {
        Command wait = Commands
            .runOnce(() -> m_drive.set(0.8))
            .andThen(
                Commands.waitUntil(this::noteDetected),
                Commands.runOnce(this::stop)
            );
            
        wait.addRequirements(this);

        return wait;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Conveyor Distance", m_drive.getPosition());
        SmartDashboard.putBoolean("Note Ready", noteDetected());
    }
}
