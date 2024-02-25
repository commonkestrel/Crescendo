package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.ShooterConstants;
import wildlib.PIDSpark;
import wildlib.utils.MathUtils;

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
            m_drive.setTargetVelocity(ShooterConstants.idleTarget);
        }, this));
    }

    /**
     * Waits for the wheels to ramp to shoot for Amp
     */
    public Command rampAmp() {
        Command shoot = Commands.runOnce(() -> m_drive.setTargetVelocity(ShooterConstants.ampTarget))
            .andThen(Commands.waitUntil(() -> MathUtils.closeEnough(m_drive.getVelocity(), ShooterConstants.ampTarget, 10)));
        shoot.addRequirements(this);

        return shoot;
    }

    /**
     * Waits for the wheels to ramp to shoot for Speaker
     */
    public Command rampSpeaker() {
        Command shoot = Commands.runOnce(() -> m_drive.setTargetVelocity(ShooterConstants.speakerTarget))
            .andThen(Commands.waitUntil(() -> MathUtils.closeEnough(m_drive.getVelocity(), ShooterConstants.speakerTarget, 10)));
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
}
