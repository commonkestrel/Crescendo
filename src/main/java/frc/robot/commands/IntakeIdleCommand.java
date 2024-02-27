package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeIdleCommand extends Command {
    private enum State {
        Running,
        Stopped,
    }

    private final Intake m_intake;

    private State m_currentState;

    public IntakeIdleCommand(Intake intake) {
        m_intake = intake;
        addRequirements(m_intake);

        m_currentState = State.Running;
    }

    @Override
    public void initialize() {
        m_intake.setSpeed(0.4);
        m_currentState = State.Running;
    }

    @Override
    public void execute() {
        switch (m_currentState) {
        case Running:
            if (!m_intake.noteDetected()) {
                m_intake.setSpeed(0.4);
                m_currentState = State.Running;
            }
        case Stopped:
            if (m_intake.noteDetected()) {
                m_intake.stop();
                m_currentState = State.Stopped;
            }
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
