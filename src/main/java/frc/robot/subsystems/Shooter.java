package frc.robot.subsystems;

import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.ShooterConstants;
import wildlib.io.PIDSpark;
import wildlib.utils.MathUtils;

/**
 * A subsystem representing the shooter on our robot
 */
public class Shooter extends SubsystemBase {
    private final PIDSpark m_flywheel;
    private final PIDSpark m_turret;

    private static Shooter m_instance;

    public static Shooter getInstance() {
        if (m_instance == null) {
            m_instance = new Shooter(
                new PIDSpark(
                    IOConstants.shooterId,
                    MotorType.kBrushless,
                    PIDSpark.SparkFlexModel(),
                    ShooterConstants.flywheelKP,
                    ShooterConstants.flywheelKI,
                    ShooterConstants.flywheelKD,
                    ShooterConstants.flywheelKF
                ),
                new PIDSpark(
                    IOConstants.turretId,
                    MotorType.kBrushless,
                    PIDSpark.SparkMaxModel(),
                    ShooterConstants.turretKP,
                    ShooterConstants.turretKI,
                    ShooterConstants.turretKD,
                    ShooterConstants.turretKF
                )
            );
        }

        return m_instance;
    }

    private Shooter(PIDSpark flywheel, PIDSpark turret) {
        m_flywheel = flywheel;
        m_flywheel.setInverted(true);
        m_flywheel.setSmartCurrentLimit(60);
        m_flywheel.burnFlash();

        m_turret = turret;
        m_turret.setPositionConversionFactor(ShooterConstants.turretPositionFactor);
        m_turret.setSmartCurrentLimit(30);
        m_turret.burnFlash();
    }

    public void initDefaultCommand() {
        setDefaultCommand(Commands.run(() -> {
            m_flywheel.set(0);
        }, this));
    }

    public void setSpeed(double speed) {
        m_flywheel.set(speed);
    }

    public REVLibError setTargetVelocity(double velocity) {
        return m_flywheel.setTargetVelocity(velocity);
    }

    public double getVelocity() {
        return m_flywheel.getVelocity();
    }

    /** Sets the target angle of the turret in radians. */
    public REVLibError setTargetTurretAngle(double angle) {
        return m_turret.setTargetPosition(angle-Math.PI/9.0);
    }

    /**
     * Waits for the wheels to ramp to shoot for Amp
     */
    public Command rampAmp() {
        Command shoot = Commands.runOnce(() -> m_flywheel.setTargetVelocity(ShooterConstants.ampTarget))
            .andThen(Commands.waitUntil(() -> MathUtils.closeEnough(m_flywheel.getVelocity(), ShooterConstants.ampTarget, 10)));
        shoot.addRequirements(this);

        return shoot;
    }

    /**
     * Waits for the wheels to ramp to shoot for Speaker
     */
    public Command rampSpeaker() {
        Command shoot = Commands.runOnce(() -> m_flywheel.setTargetVelocity(ShooterConstants.speakerTarget))
            .andThen(Commands.waitUntil(() -> MathUtils.closeEnough(m_flywheel.getVelocity(), ShooterConstants.speakerTarget, 10)));
        shoot.addRequirements(this);

        return shoot;
    }

    /**
     * Waits for the note to be shot.
     * 
     * @return A command that waits for the note to shoot that requires this instance.
     */
    public Command waitForShoot() {
        Command waitForShoot = Commands.waitSeconds(ShooterConstants.shootTime);
        waitForShoot.addRequirements(this);

        return waitForShoot;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter Current", m_flywheel.getOutputCurrent());
        SmartDashboard.putNumber("Shooter Velocity", m_flywheel.getVelocity());
        SmartDashboard.putNumber("Shooter Output", m_flywheel.getAppliedOutput());
    }
}
