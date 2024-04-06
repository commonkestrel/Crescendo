package frc.robot.commands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Leds;

public class IntakeIdleCommand extends Command {
    private enum State {
        Running,
        Stopped,
    }

    private final Intake m_intake;
    private final Leds m_leds;

    private State m_currentState;

    public IntakeIdleCommand(Intake intake, Leds leds) {
        m_intake = intake;
        m_leds = leds;
        addRequirements(m_intake);

        m_currentState = State.Running;
    }

    @Override
    public void initialize() {
        if (m_intake.noteDetected()) {
            m_currentState = State.Stopped;
            m_intake.stop();
        } else {
            m_intake.setSpeed(0.4);
            m_currentState = State.Running;
        }
    }

    @Override
    public void execute() {
        switch (m_currentState) {
        case Running:
            if (m_intake.noteDetected()) {
                m_intake.setSpeed(0.0);
                m_currentState = State.Stopped;
                m_leds.flash(Color.kGreen);
            }
            break;
        case Stopped:
            if (!m_intake.noteDetected()) {
                m_intake.setSpeed(0.4);
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
