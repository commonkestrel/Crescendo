package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter;

/**
 * Command that ramps the shooter to Speaker speed.
 * Can be used to ramp during travel to reduce cycle time.
 */
public class RampSpeakerCommand extends Command {
    private final Shooter m_shooter;

    public RampSpeakerCommand(Shooter shooter) {
        m_shooter = shooter;
        addRequirements(m_shooter);
    }

    @Override
    public void initialize() {
        m_shooter.setTargetVelocity(ShooterConstants.speakerTarget);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
