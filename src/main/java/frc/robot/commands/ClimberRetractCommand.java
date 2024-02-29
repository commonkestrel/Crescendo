package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.Climber;
import wildlib.utils.MathUtils;

public class ClimberRetractCommand extends Command {
    private enum State {
        Seeking,
        LeftCollision,
        RightCollision,
        Matched,
    }

    private final Climber m_climber;
    private State m_currentState;

    public ClimberRetractCommand(Climber climber) {
        m_climber = climber;
        addRequirements(m_climber);

        m_currentState = State.Seeking;
    }

    @Override
    public void initialize() {
        m_climber.setLeftTargetPosition(ClimberConstants.retractedPosition);
        m_climber.setRightTargetPosition(ClimberConstants.retractedPosition);
        m_currentState = State.Seeking;
    }

    @Override
    public void execute() {
        final boolean leftSpiked = m_climber.getLeftCurrent() > ClimberConstants.spikeCurrent;
        final boolean rightSpiked = m_climber.getRightCurrent() > ClimberConstants.spikeCurrent;

        switch (m_currentState) {
        case Seeking:
            if (leftSpiked && rightSpiked) {
                m_currentState = State.Matched;
            } else if (leftSpiked) {
                m_climber.stopLeft();
                m_currentState = State.LeftCollision;
            } else if (rightSpiked) {
                m_climber.stopRight();
                m_currentState = State.RightCollision;
            }

            break;
        case RightCollision:
            if (leftSpiked) {
                m_climber.setRightTargetPosition(ClimberConstants.retractedPosition);
                m_currentState = State.Matched;
            }

            break;
        case LeftCollision:
            if (rightSpiked) {
                m_climber.setLeftTargetPosition(ClimberConstants.retractedPosition);
                m_currentState = State.Matched;
            }

            break;
        case Matched:
            break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_climber.stopLeft();
        m_climber.stopRight();
    }

    @Override
    public boolean isFinished() {
        return MathUtils.closeEnough(m_climber.getLeftPosition(), ClimberConstants.retractedPosition, 0.25)
            && MathUtils.closeEnough(m_climber.getRightPosition(), ClimberConstants.retractedPosition, 0.25);
    }
}
