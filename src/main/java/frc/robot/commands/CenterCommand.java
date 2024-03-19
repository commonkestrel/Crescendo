package frc.robot.commands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Leds;
import frc.robot.subsystems.Leds.LedState;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.drive.Swerve;

public class CenterCommand extends Command {
    private enum State {
        Search,
        Found,
    }

    private State m_currentState;
    private final Limelight m_limelight;
    private final Leds m_leds;
    private final Swerve m_swerve;
    private final CenterAmpCommand m_ampCommand;
    private final CenterSpeakerCommand m_speakerCommand;
    private Command m_currentCommand;

    public CenterCommand(Swerve swerve, Limelight limelight, Leds leds) {
        m_limelight = limelight;
        m_leds = leds;
        m_swerve = swerve;
        m_ampCommand = new CenterAmpCommand(swerve, m_limelight, leds);
        m_speakerCommand = new CenterSpeakerCommand(swerve, limelight, leds);
    }

    @Override
    public void initialize() {
        m_limelight.setPipelineIndex(AutoConstants.ampPipeline);
        m_currentCommand = null;
        
        if (m_limelight.getTV()) {
            initFound();
            m_currentState = State.Found;
        } else {
            m_currentState = State.Search;
        }

        m_leds.set(LedState.kFade, Color.kMagenta);

    }

    @Override
    public void execute() {
        System.out.println(m_limelight.getTV());
        switch (m_currentState) {
        case Search:
            if (m_limelight.getTV()) {
                initFound();
                m_currentState = State.Found;
            } else {
                m_swerve.drive(0.0, 0.0, 0.2, false, true);
            }

            break;
        case Found:
            m_currentCommand.execute();
            break;
        }
    }

    @Override
    public boolean isFinished() {
        return m_currentState == State.Found && m_currentCommand.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        if (m_currentCommand != null) m_currentCommand.end(interrupted);
    }

    private void initFound() {
        switch ((int) m_limelight.getFiducialID()) {
        case 5:
        case 6:
            m_currentCommand = m_ampCommand;
        default:
            m_currentCommand = m_speakerCommand;
        }

        m_currentCommand.initialize();
    }
}
