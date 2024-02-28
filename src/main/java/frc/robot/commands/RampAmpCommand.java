package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter;

/**
 * Command that ramps the shooter to Amp speed
 * Can be used to ramp during travel to reduce cycle time
 */
public class RampAmpCommand extends Command {
    private final Shooter m_shooter;

    public RampAmpCommand(Shooter shooter) {
        m_shooter = shooter;
        addRequirements(m_shooter);
    }

    @Override
    public void initialize() {
        m_shooter.setSpeed(0.25);
    }

    @Override
    public boolean isFinished() {
        return m_shooter.getVelocity() >= ShooterConstants.ampTarget;
    }
}
