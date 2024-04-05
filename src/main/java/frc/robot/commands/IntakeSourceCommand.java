package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class IntakeSourceCommand extends Command {
    enum State {
        notPassed,
        Passed,
        Equilibrium
    }
    private final Intake m_intake;
    private final Shooter m_shooter;

    private boolean m_noteDetected;
    private State m_state;

    public IntakeSourceCommand(Intake intake, Shooter shooter) {
        m_intake = intake; 
        m_shooter = shooter;
        m_noteDetected = false;
        addRequirements(m_intake, m_shooter);
    }

    @Override
    public void initialize() {
        m_intake.setPrerollerSpeed(0.0);
        m_intake.setTargetIndexerVelocity(-1000);
        m_shooter.setTargetVelocity(ShooterConstants.intakeTarget);
        m_noteDetected = m_intake.noteDetected();
        m_state = State.notPassed;
    }

    @Override
    public void execute() {
        switch (m_state) {
            case notPassed:
        if (m_intake.noteDetected()) {
            m_noteDetected = true;
        } else if (!m_intake.noteDetected() && m_noteDetected) {
            m_shooter.setSpeed(0.0);
            m_state = State.Passed;
        }
            break;
            case Passed:
            m_intake.setIndexerSpeed(0.1);
        if (m_intake.noteDetected()) {
            m_state = State.Equilibrium;
        }
            break;
            case Equilibrium:
                break;
    }
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.stop();
    }

    @Override
    public boolean isFinished() {
        // Finished if we've seen a Note and it has passed the sensor completely
        return m_state == State.Equilibrium;
    }
}
