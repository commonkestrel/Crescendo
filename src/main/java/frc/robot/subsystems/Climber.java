package frc.robot.subsystems;

import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.IOConstants;
import wildlib.PIDSpark;

import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    private static Climber m_instance;

    private final PIDSpark m_left;
    private final PIDSpark m_right;

    public static Climber getInstance() {
        if (m_instance == null) {
            m_instance = new Climber(
                new PIDSpark(
                    IOConstants.climbLeftId,
                    MotorType.kBrushless,
                    PIDSpark.SparkMaxModel(),
                    ClimberConstants.leftKP,
                    ClimberConstants.leftKI,
                    ClimberConstants.leftKD
                ),
                new PIDSpark(
                    IOConstants.climbRightId,
                    MotorType.kBrushless,
                    PIDSpark.SparkMaxModel(),
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

        m_left.setSoftLimit(SoftLimitDirection.kReverse, ClimberConstants.leftMinPosition);
        m_left.setSoftLimit(SoftLimitDirection.kForward, ClimberConstants.maxPosition);

        m_left.setPositionConversionFactor(ClimberConstants.conversionFactor);
        m_left.setIdleMode(IdleMode.kBrake);

        m_left.burnFlash();

        m_right.setSoftLimit(SoftLimitDirection.kReverse, ClimberConstants.rightMinPosition);
        m_right.setSoftLimit(SoftLimitDirection.kForward, ClimberConstants.maxPosition);

        m_right.setPositionConversionFactor(ClimberConstants.conversionFactor);
        m_right.setIdleMode(IdleMode.kBrake);

        enableSoftLimits();

        m_right.burnFlash();
    }

    public REVLibError setLeftTargetPosition(double position) {
        return m_left.setTargetPosition(position);
    }

    public REVLibError setRightTargetPosition(double position) {
        return m_right.setTargetPosition(position);
    }

    public void setLeft(double speed) {
        m_left.set(speed);
    }

    public void setRight(double speed) {
        m_right.set(speed);
    }

    public void disableFowardSoftLimit() {
        m_left.enableSoftLimit(SoftLimitDirection.kForward, false);
        m_right.enableSoftLimit(SoftLimitDirection.kForward, false);
    }

    public void enableFowardSoftLimit() {
        m_left.getEncoder().setPosition(0.0);
        m_right.getEncoder().setPosition(0.0);
        m_left.enableSoftLimit(SoftLimitDirection.kForward, true);
        m_right.enableSoftLimit(SoftLimitDirection.kForward, true);
    }

    public void zeroPosition() {
        m_left.getEncoder().setPosition(0.0);
        m_right.getEncoder().setPosition(0.0);
    }

    public void enableSoftLimits() {
        m_left.enableSoftLimit(SoftLimitDirection.kForward, true);
        m_right.enableSoftLimit(SoftLimitDirection.kForward, true);
        m_left.enableSoftLimit(SoftLimitDirection.kReverse, true);
        m_right.enableSoftLimit(SoftLimitDirection.kReverse, true);
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

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climber Left Position", m_left.getPosition());
        SmartDashboard.putNumber("Climber Right Position", m_right.getPosition());
    }
}
