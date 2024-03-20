package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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
    private final CenterTargetCommand m_ampCommand;
    private final CenterTargetCommand m_sourceCommand;
    private final CenterSpeakerCommand m_speakerCommand;
    private Command m_currentCommand;

    public CenterCommand(Swerve swerve, Limelight limelight, Leds leds, XboxController xboxController) {
        m_limelight = limelight;
        m_leds = leds;
        m_swerve = swerve;
        m_ampCommand = new CenterTargetCommand(swerve, m_limelight, leds, AutoConstants.ampDistance);
        m_sourceCommand = new CenterTargetCommand(swerve, limelight, leds, AutoConstants.sourceDistance);
        m_speakerCommand = new CenterSpeakerCommand(swerve, limelight, leds, xboxController);
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
        System.out.println((int) m_limelight.getFiducialID());
        switch ((int) m_limelight.getFiducialID()) {
        case 5:
        case 6:
            m_currentCommand = m_ampCommand;
            break;
        case 3:
        case 4:
        case 7:
        case 8:
            m_currentCommand = m_speakerCommand;
            break;
        default:
            m_currentCommand = m_sourceCommand;
            break;
        }

        m_currentCommand.initialize();
    }
}
