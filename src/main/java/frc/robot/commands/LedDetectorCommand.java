package frc.robot.commands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Leds;
import frc.robot.subsystems.Leds.LedState;

public class LedDetectorCommand extends Command {
    private final Leds m_leds;
    private final Intake m_intake;
    private boolean m_previousDetected;

    public LedDetectorCommand(Intake intake, Leds leds) {
        m_leds = leds;
        m_intake = intake;
        m_leds.set(LedState.kSolid, m_intake.noteDetected() ? Color.kGreen : Color.kRed);

        addRequirements(m_leds);
    }

    @Override
    public void initialize() {
        m_previousDetected = m_intake.noteDetected();
        m_leds.set(LedState.kSolid, m_intake.noteDetected() ? Color.kGreen : Color.kRed);
    }

    @Override
    public void execute() {
        if (m_previousDetected != m_intake.noteDetected()) {
            m_previousDetected = m_intake.noteDetected();
            m_leds.set(LedState.kSolid, m_intake.noteDetected() ? Color.kGreen : Color.kRed);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
