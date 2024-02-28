package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import wildlib.utils.MathUtils;

public class ShootAmpCommand extends Command {
    private enum State {
        Feed,
        RevUp,
        Shoot,
    }

    private final Intake m_intake;
    private final Shooter m_shooter;
    
    private State m_currentState;

    public ShootAmpCommand(Intake intake, Shooter shooter) {
        m_intake = intake;
        m_shooter = shooter;
        addRequirements(m_intake, m_shooter);

        m_currentState = State.Feed;
    }

    @Override
    public void initialize() {
        m_shooter.setTargetVelocity(ShooterConstants.ampTarget);
        m_intake.setSpeed(0.4);
    }

    @Override
    public void execute() {
        switch (m_currentState) {
        case Feed:
            if (m_intake.noteDetected()) {
                m_intake.setSpeed(0.0);
                m_currentState = State.RevUp;
            }
            break;
        case RevUp:
            if (MathUtils.closeEnough(m_shooter.getVelocity(), ShooterConstants.ampTarget, 10.0)) {
                m_intake.setSpeed(0.4);
                m_currentState = State.Shoot;
            }
            break;
        case Shoot:
            break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.setTargetVelocity(ShooterConstants.idleTarget);
    }

    @Override
    public boolean isFinished() {
        return m_currentState == State.Shoot && !m_intake.noteDetected();
    }
}
