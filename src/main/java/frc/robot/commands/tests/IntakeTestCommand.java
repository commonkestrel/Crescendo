package frc.robot.commands.tests;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Leds;
import wildlib.testing.SystemTestCommand;
import wildlib.utils.MathUtils;

public class IntakeTestCommand extends SystemTestCommand {
    private enum State {
        kStopping,
        kAmp,
        kSpeaker,
        kFinished,
    }

    private final Intake m_intake;
    private State m_currentState;

    public IntakeTestCommand(Intake intake) {
        m_intake = intake;
        addRequirements(m_intake);
    }

    @Override
    public void initializeTest() {
        m_intake.stop();
        m_currentState = State.kStopping;
    }

    @Override
    public void executeTest() {
        switch (m_currentState) {
        case kStopping:
            if (MathUtils.closeEnough(m_intake.getIndexerVelocity(), 0.0, 5.0)) {
                m_intake.setTargetIndexerVelocity(IntakeConstants.ampTarget);
                m_currentState = State.kAmp;
            }
            break;
        case kAmp:
            if (MathUtils.closeEnough(m_intake.getIndexerVelocity(), IntakeConstants.ampTarget + 50, 100.0)) {
                m_intake.setTargetIndexerVelocity(IntakeConstants.speakerTarget);
                m_currentState = State.kSpeaker;
            }
            break;
        case kSpeaker:
            if (MathUtils.closeEnough(m_intake.getIndexerVelocity(), IntakeConstants.speakerTarget + 50, 100.0)) {
                m_intake.stop();
                m_currentState = State.kFinished;
            }
            break;
        default:
            break;
        }
    }

    @Override
    public void endTest(boolean interrupted) {
        m_intake.stop();
    }

    @Override 
    public boolean isTestFinished() {
        return m_currentState == State.kFinished;
    }

    @Override
    public boolean isTestSuccessful() {
        return isFinished();
    }
}
