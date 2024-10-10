package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.Climber;
import wildlib.utils.MathUtils;

public class ClimberReleaseCommand extends Command {
    private final Climber m_climber;
    public enum Side {
        Left,
        Right,
        Both
    }
    private Side m_side;

    public ClimberReleaseCommand(Climber climber, frc.robot.commands.ClimberRetractCommand.Side side) {
        m_climber = climber;
        switch (side) {
            case Left:
            m_side = Side.Left;
            break;
            case Right:
            m_side = Side.Right;
            break;
            case Both:
            m_side = Side.Both;
            break;
        }
        addRequirements(m_climber);
    }

    @Override
    public void initialize() {
        switch (m_side) {
            case Left:
            m_climber.setLeft(1.0);
            break;
            case Right:
            m_climber.setRight(1.0);
            break;
            case Both:
            m_climber.setLeft(1.0);
            m_climber.setRight(1.0);
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
        return MathUtils.closeEnough(m_climber.getLeftPosition(), ClimberConstants.extendedPosition, 0.25)
            && MathUtils.closeEnough(m_climber.getRightPosition(), ClimberConstants.extendedPosition, 0.25);
    }
}
