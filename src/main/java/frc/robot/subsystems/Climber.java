package frc.robot.subsystems;

import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.IOConstants;

import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

    public REVLibError setLeftTargetPosition(double position) {
        return m_left.setTargetPosition(position);
    }

    public REVLibError setRightTargetPosition(double position) {
        return m_right.setTargetPosition(position);
    }

    public double getLeftPosition() {
        return m_left.getPosition();
    }

    public double getRightPosition() {
        return m_right.getPosition();
    }

    public void stopLeft() {
        m_left.stopMotor();
    }

    public void stopRight() {
        m_right.stopMotor();
    }
}
