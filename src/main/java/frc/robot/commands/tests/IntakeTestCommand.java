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
    private final Leds m_leds;
    private State m_currentState;

    public IntakeTestCommand(Intake intake, Leds leds) {
        m_intake = intake;
        m_leds = leds;
        addRequirements(m_intake);
    }

    @Override
    public void initializeTest() {
        m_intake.setTargetVelocity(0);
        m_currentState = State.kStopping;
    }

    @Override
    public void executeTest() {
        switch (m_currentState) {
        case kStopping:
            if (MathUtils.closeEnough(m_intake.getVelocity(), 0.0, 5.0)) {
                m_intake.setTargetVelocity(IntakeConstants.ampTarget);
                m_currentState = State.kAmp;
            }
            break;
        case kAmp:
            if (MathUtils.closeEnough(m_intake.getVelocity(), IntakeConstants.ampTarget + 50, 100.0)) {
                m_intake.setTargetVelocity(IntakeConstants.speakerTarget);
                m_currentState = State.kSpeaker;
            }
            break;
        case kSpeaker:
            if (MathUtils.closeEnough(m_intake.getVelocity(), IntakeConstants.speakerTarget + 50, 100.0)) {
                m_intake.setSpeed(0.0);
                m_currentState = State.kFinished;
                m_leds.flash(Color.kBlue);
            }
            break;
        default:
            break;
        }
    }

    @Override
    public void endTest(boolean interrupted) {
        m_intake.setSpeed(0.0);
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
