package frc.robot.commands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Leds;
import frc.robot.subsystems.Leds.LedState;
import wildlib.utils.MathUtils;

public class ClimberRetractCommand extends Command {
    // private enum State {
    //     Seeking,
    //     LeftCollision,
    //     RightCollision,
    //     Matched,
    // }

    public enum Side {
        Left,
        Right,
        Both
    }

    private final Climber m_climber;
    private final Leds m_leds;
    // private State m_currentState;
    private Side m_side;

    public ClimberRetractCommand(Climber climber, Leds leds, Side side) {
        m_climber = climber;
        m_leds = leds;
        addRequirements(m_climber);

        m_side = side;
    }

    @Override
    public void initialize() {
        // m_climber.setLeft(-1.0);
        // m_climber.setRight(-1.0);
        m_leds.set(LedState.kRainbow, Color.kBlack);
        switch (m_side) {
            case Left:
            m_climber.setLeft(-1.0);
            break;
            case Right:
            m_climber.setRight(-1.0);
            break;
            case Both:
            m_climber.setLeft(-1.0);
            m_climber.setRight(-1.0);
            break;
        }
        //m_currentState = State.Seeking;
    }

    @Override
    public void execute() {
        // final boolean leftSpiked = m_climber.getLeftCurrent() > ClimberConstants.spikeCurrent;
        // final boolean rightSpiked = m_climber.getRightCurrent() > ClimberConstants.spikeCurrent;

        // switch (m_currentState) {
        // case Seeking:
        //     if (leftSpiked && rightSpiked) {
        //         m_currentState = State.Matched;
        //     } else if (leftSpiked) {
        //         m_climber.stopLeft();
        //         m_currentState = State.LeftCollision;
        //     } else if (rightSpiked) {
        //         m_climber.stopRight();
        //         m_currentState = State.RightCollision;
        //     }
        //     printPos();

        //     break;
        // case RightCollision:
        //     if (leftSpiked) {
        //         m_climber.setRightTargetPosition(ClimberConstants.retractedPosition);
        //         m_currentState = State.Matched;
        //     }
        //     printPos();
        //     break;
        // case LeftCollision:
        //     if (rightSpiked) {
        //         m_climber.setLeftTargetPosition(ClimberConstants.retractedPosition);
        //         m_currentState = State.Matched;
        //     }
        //     printPos();
        //     break;
        // case Matched:
        // printPos();
        //     break;
        // }
    }

    public void printPos() {
        System.out.printf("Left Climber Position: %f; Right Climber Position: %f%n", m_climber.getLeftPosition(), m_climber.getRightPosition());
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
