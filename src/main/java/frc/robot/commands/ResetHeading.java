package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Swerve;

public class ResetHeading extends Command {
    private final Swerve m_drive;
    private final double m_reset;

    public ResetHeading(Swerve drive, double resetRadians) {
        m_drive = drive;
        m_reset = resetRadians;
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        m_drive.resetHeading(m_reset);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
