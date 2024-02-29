package frc.robot.subsystems;

import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.ShooterConstants;
import wildlib.PIDSpark;
import wildlib.utils.MathUtils;

/**
 * A subsystem representing the shooter on our robot
 */
public class Shooter extends SubsystemBase {
    private final PIDSpark m_motor;

    private static Shooter m_instance;

    public static Shooter getInstance() {
        if (m_instance == null) {
            m_instance = new Shooter(new PIDSpark(
                IOConstants.shooterId,
                MotorType.kBrushless,
                ShooterConstants.motorKP,
                ShooterConstants.motorKI,
                ShooterConstants.motorKD,
                ShooterConstants.motorKF
            ));
        }

        return m_instance;
    }

    private Shooter(PIDSpark drive) {
        m_motor = drive;
        m_motor.setInverted(true);
    }

    public void initDefaultCommand() {
        setDefaultCommand(Commands.run(() -> {
            m_motor.set(0);
        }, this));
    }

    public void setSpeed(double speed) {
        m_motor.set(speed);
    }

    public REVLibError setTargetVelocity(double velocity) {
        return m_motor.setTargetVelocity(velocity);
    }

    public double getVelocity() {
        return m_motor.getVelocity();
    }

    /**
     * Waits for the wheels to ramp to shoot for Amp
     */
    public Command rampAmp() {
        Command shoot = Commands.runOnce(() -> m_motor.setTargetVelocity(ShooterConstants.ampTarget))
            .andThen(Commands.waitUntil(() -> MathUtils.closeEnough(m_motor.getVelocity(), ShooterConstants.ampTarget, 10)));
        shoot.addRequirements(this);

        return shoot;
    }

    /**
     * Waits for the wheels to ramp to shoot for Speaker
     */
    public Command rampSpeaker() {
        Command shoot = Commands.runOnce(() -> m_motor.setTargetVelocity(ShooterConstants.speakerTarget))
            .andThen(Commands.waitUntil(() -> MathUtils.closeEnough(m_motor.getVelocity(), ShooterConstants.speakerTarget, 10)));
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
        SmartDashboard.putNumber("Shooter Velocity", m_motor.getVelocity());
    }
}
