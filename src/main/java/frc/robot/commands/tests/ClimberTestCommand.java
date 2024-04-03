package frc.robot.commands.tests;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Leds;
import wildlib.testing.SystemTestCommand;
import wildlib.utils.MathUtils;

public class ClimberTestCommand extends SystemTestCommand {
    private enum State {
        kExtending,
        kRetracting,
        kFinished,
    }

    private final Climber m_climber;
    private final Leds m_leds;
    private State m_currentState;

    public ClimberTestCommand(Climber climber, Leds leds) {
        m_climber = climber;
        m_leds = leds;
        addRequirements(m_climber);
    }

    @Override
    public void initializeTest() {
        m_climber.zeroRetracted();
        m_currentState = State.kExtending;
        m_climber.setLeftTargetPosition(ClimberConstants.extendedPosition);
        m_climber.setRightTargetPosition(ClimberConstants.extendedPosition);
    }

    @Override
    public void executeTest() {
        switch (m_currentState) {
        case kExtending:
            if (MathUtils.closeEnough(m_climber.getLeftPosition(), ClimberConstants.extendedPosition, 1.0) && MathUtils.closeEnough(m_climber.getRightPosition(), ClimberConstants.extendedPosition, 1.0)) {
                m_climber.setLeftTargetPosition(ClimberConstants.retractedPosition);
                m_climber.setRightTargetPosition(ClimberConstants.retractedPosition);
                m_currentState = State.kRetracting;
            }
            break;
        case kRetracting:
            if (MathUtils.closeEnough(m_climber.getLeftPosition(), ClimberConstants.retractedPosition, 1.0) && MathUtils.closeEnough(m_climber.getRightPosition(), ClimberConstants.retractedPosition, 1.0)) {
                m_climber.setLeft(0.0);
                m_climber.setRight(0.0);
                m_currentState = State.kFinished;
                m_leds.flash(Color.kBlue);
            }
            break;
        case kFinished:
            break;
        }
    }

    @Override
    public void endTest(boolean interrupted) {
        m_climber.setLeft(0.0);
        m_climber.setRight(0.0);
    }

    @Override
    public boolean isTestFinished() {
        return m_currentState == State.kFinished;
    }
}
