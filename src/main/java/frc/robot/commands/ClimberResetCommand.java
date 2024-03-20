package frc.robot.commands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Leds;
import frc.robot.subsystems.Leds.LedState;
import wildlib.utils.MathUtils;

public class ClimberResetCommand extends Command{
    private final Climber m_climber;
    private final Leds m_leds;

    public ClimberResetCommand(Climber climber, Leds leds) {
        m_climber = climber;
        m_leds = leds;
        addRequirements(m_climber);
    }

    @Override
    public void initialize() {
        m_leds.set(LedState.kRainbowFast, Color.kBeige);
        m_climber.disableFowardSoftLimit();
        m_climber.setLeftTargetPosition(-ClimberConstants.retractedPosition);
        m_climber.setRightTargetPosition(-ClimberConstants.retractedPosition);
    }

    @Override
    public void end(boolean interrupted) {
        m_climber.stopLeft();
        m_climber.stopRight();
        m_climber.enableFowardSoftLimit();
    }

    @Override
    public boolean isFinished() {
        return MathUtils.closeEnough(m_climber.getLeftPosition(), -ClimberConstants.retractedPosition, 0.25)
            && MathUtils.closeEnough(m_climber.getRightPosition(), -ClimberConstants.retractedPosition, 0.25);
    }
}
