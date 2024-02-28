package frc.robot.subsystems;

import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.IOConstants;
import wildlib.PIDSpark;

import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    private static Climber m_instance;

    private final PIDSpark m_left;
    private final PIDSpark m_right;

    public static Climber getInstance() {
        if (m_instance == null) {
            m_instance = new Climber(
                new PIDSpark(IOConstants.climbLeftId,
                    MotorType.kBrushless,
                    ClimberConstants.leftKP,
                    ClimberConstants.leftKI,
                    ClimberConstants.leftKD
                ),
                new PIDSpark(IOConstants.climbRightId,
                    MotorType.kBrushless,
                    ClimberConstants.rightKP,
                    ClimberConstants.rightKI,
                    ClimberConstants.rightKD
                )
            );
        }

        return m_instance;
    }

    private Climber(PIDSpark leftMotor, PIDSpark rightMotor) {
        m_left = leftMotor;
        m_right = rightMotor;

        m_left.setSoftLimit(SoftLimitDirection.kReverse, ClimberConstants.minPosition);
        m_left.setSoftLimit(SoftLimitDirection.kForward, ClimberConstants.maxPosition);

        m_left.setPositionConversionFactor(ClimberConstants.conversionFactor);
        m_left.setIdleMode(IdleMode.kBrake);

        m_right.setSoftLimit(SoftLimitDirection.kReverse, ClimberConstants.minPosition);
        m_right.setSoftLimit(SoftLimitDirection.kForward, ClimberConstants.maxPosition);

        m_right.setPositionConversionFactor(ClimberConstants.conversionFactor);
        m_right.setIdleMode(IdleMode.kBrake);
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

    public double getLeftCurrent() {
        return m_left.getOutputCurrent();
    }

    public double getRightCurrent() {
        return m_right.getOutputCurrent();
    }
}
