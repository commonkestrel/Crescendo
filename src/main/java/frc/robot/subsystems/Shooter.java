package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.ShooterConstants;
import wildlib.PIDSpark;

// TODO: Construct skeleton
/**
 * A subsystem representing the shooter on our robot
 */
public class Shooter extends SubsystemBase {
    private final PIDSpark m_drive;

    private static Shooter m_instance;

    public static Shooter getInstance() {
        if (m_instance == null) {
            m_instance = new Shooter(new PIDSpark(
                IOConstants.shooterId,
                MotorType.kBrushless,
                ShooterConstants.driveKP,
                ShooterConstants.driveKI,
                ShooterConstants.driveKD
            ));
        }

        return m_instance;
    }

    private Shooter(PIDSpark drive) {
        m_drive = drive;
    }

    public void initDefaultCommand() {
        setDefaultCommand(Commands.run(() -> {
            m_drive.setTargetVelocity(ShooterConstants.targetVelocity);
        }, this));
    }
}
