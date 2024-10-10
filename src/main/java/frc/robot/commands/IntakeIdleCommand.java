package frc.robot.commands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Leds;

public class IntakeIdleCommand extends Command {
    private enum State {
        Running,
        DebounceR,
        DebounceF,
        Equilibrium,
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
        if (m_intake.noteDetected()) {
            m_currentState = State.Equilibrium;
            m_intake.stop();
        } else {
            m_intake.setIndexerSpeed(0.4);
            m_intake.setPrerollerSpeed(1.0);
            m_currentState = State.Running;
        }
    }

    @Override
    public void execute() {
        if (m_intake.getOverride() && !(m_currentState == State.Equilibrium)){
            m_intake.stop();
            m_currentState = State.Equilibrium;
        }
        switch (m_currentState) {
        case Running:
            if (m_intake.noteDetected()) {
                m_currentState = State.DebounceR;
                m_intake.setIndexerSpeed(-0.175);
            }
            break;
        case DebounceR:
            if (!m_intake.noteDetected()) {
                m_intake.setIndexerSpeed(0.1);
                m_currentState = State.DebounceF;
            }
            break;
        case DebounceF:
            if (m_intake.noteDetected()) {
                m_intake.stop();
                m_currentState = State.Equilibrium;
            }
            break;
        case Equilibrium:
            if (!m_intake.noteDetected()) {
                m_intake.setIndexerSpeed(0.4);
                m_intake.setPrerollerSpeed(1.0);
                m_currentState = State.Running;
            }
            break;
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
