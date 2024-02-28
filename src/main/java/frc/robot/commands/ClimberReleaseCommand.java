package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.Climber;
import wildlib.utils.MathUtils;

public class ClimberReleaseCommand extends Command {
    private final Climber m_climber;

    public ClimberReleaseCommand(Climber climber) {
        m_climber = climber;
        addRequirements(m_climber);
    }

    @Override
    public void initialize() {
        m_climber.setLeftTargetPosition(ClimberConstants.extendedPosition);
        m_climber.setRightTargetPosition(ClimberConstants.extendedPosition);
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
