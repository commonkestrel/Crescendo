package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends Command {
    private final Intake m_intake;

    public IntakeCommand(Intake intake) {
        m_intake = intake;
        addRequirements(m_intake);
    }

    @Override
    public void initialize() {
        m_intake.setIndexerSpeed(0.3);
        m_intake.setPrerollerSpeed(0.6);
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
