package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

/**
 * Command to feed from the intake to the shooter as an Amp shot
 * Relies on the shooter already being spun up to speed through something like {@link RampAmpCommand}
 */
public class RampedAmpCommand extends Command {
    private enum State {
        Feed,
        Wait,
    }

    private final Shooter m_shooter;
    private final Intake m_intake;

    private State m_currentState;
    private long m_lastDetected;

    public RampedAmpCommand(Shooter shooter, Intake intake) {
        m_shooter = shooter;
        m_intake = intake;

        addRequirements(m_shooter, m_intake);
    }

    @Override
    public void initialize() {
        m_intake.setSpeed(0.4);
        m_shooter.setSpeed(0.25);
    }

    @Override
    public void execute() {
        switch (m_currentState) {
        case Feed:
            if (!m_intake.noteDetected()) {
                m_lastDetected = System.nanoTime();
                m_currentState = State.Wait;
            }
            break;
        case Wait:
            break;
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        m_shooter.setSpeed(0.0);
    }

    @Override
    public boolean isFinished() {
        // Wait one second after the note is last detected before ending
        return m_currentState == State.Wait && System.nanoTime() - m_lastDetected > 1e9;
    }
}
