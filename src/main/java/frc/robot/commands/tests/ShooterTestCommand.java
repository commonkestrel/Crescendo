package frc.robot.commands.tests;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Leds;
import frc.robot.subsystems.Shooter;
import wildlib.testing.SystemTestCommand;
import wildlib.utils.MathUtils;

public class ShooterTestCommand extends SystemTestCommand {
    private enum State {
        kStopping,
        kAmp,
        kSpeaker,
        kFinished,
    }

    private final Shooter m_shooter;
    private final Leds m_leds;
    private State m_currentState;

    public ShooterTestCommand(Shooter shooter, Leds leds) {
        m_shooter = shooter;
        m_leds = leds;
        addRequirements(m_shooter);
    }

    @Override
    public void initializeTest() {
        m_shooter.setTargetVelocity(0);
        m_currentState = State.kStopping;
    }

    @Override
    public void executeTest() {
        switch (m_currentState) {
        case kStopping:
            if (MathUtils.closeEnough(m_shooter.getVelocity(), 0.0, 5.0)) {
                m_shooter.setTargetVelocity(ShooterConstants.ampTarget);
                m_currentState = State.kAmp;
            }
            break;
        case kAmp:
            if (MathUtils.closeEnough(m_shooter.getVelocity(), ShooterConstants.ampTarget + 50, 100.0)) {
                m_shooter.setTargetVelocity(ShooterConstants.speakerTarget);
                m_currentState = State.kSpeaker;
            }
            break;
        case kSpeaker:
            if (MathUtils.closeEnough(m_shooter.getVelocity(), ShooterConstants.speakerTarget + 50, 100.0)) {
                m_shooter.setSpeed(0.0);
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
        m_shooter.setSpeed(0.0);
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
