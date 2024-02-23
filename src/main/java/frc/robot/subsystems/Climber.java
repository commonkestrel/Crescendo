package frc.robot.subsystems;

import frc.robot.Constants.IOConstants;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import wildlib.NotYetImplemented;
import wildlib.PIDSpark;

public class Climber extends SubsystemBase {
    private static Climber m_instance;

    private final PIDSpark m_left;
    private final PIDSpark m_right;

    public static Climber getInstance() {
        if (m_instance == null) {
            m_instance = new Climber(
                new PIDSpark(IOConstants.climbLeftId, MotorType.kBrushless), 
                new PIDSpark(IOConstants.climbRightId, MotorType.kBrushless)
            );
        }

        return m_instance;
    }

    private Climber(PIDSpark leftMotor, PIDSpark rightMotor) {
        m_left = leftMotor;
        m_right = rightMotor;
    }

    // TODO: Create a command that allows us to climb and hang
    public Command climb() {
        throw new NotYetImplemented();
    }
}
