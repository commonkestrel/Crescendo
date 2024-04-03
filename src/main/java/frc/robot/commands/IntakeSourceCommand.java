package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class IntakeSourceCommand extends Command {
    private final Intake m_intake;
    private final Shooter m_shooter;

    private boolean m_noteDetected;

    public IntakeSourceCommand(Intake intake, Shooter shooter) {
        m_intake = intake; 
        m_shooter = shooter;
        m_noteDetected = false;
        addRequirements(m_intake, m_shooter);
    }

    @Override
    public void initialize() {
        m_intake.setTargetIndexerVelocity(IntakeConstants.idleIndexerTarget);
        m_intake.setTargetPrerollerVelocity(IntakeConstants.idlePrerollerTarget);
        m_shooter.setTargetVelocity(ShooterConstants.intakeTarget);
        m_noteDetected = m_intake.noteDetected();
    }

    @Override
    public void execute() {
        if (m_intake.noteDetected()) {
            m_noteDetected = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.stop();
        m_shooter.setSpeed(0.0);
    }

    @Override
    public boolean isFinished() {
        // Finished if we've seen a Note and it has passed the sensor completely
        return m_noteDetected && !m_intake.noteDetected();
    }
}
