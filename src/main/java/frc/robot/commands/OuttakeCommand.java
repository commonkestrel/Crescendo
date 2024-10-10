package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class OuttakeCommand extends Command {
    private enum State {
        kRamping,
        kOuttake,
    }

    private final Intake m_intake;
    private State m_currentState;

    public OuttakeCommand(Intake intake) {
        m_intake = intake;
        addRequirements(m_intake);
    }

    @Override
    public void initialize() {
        m_intake.setPrerollerSpeed(-1.0);
        m_currentState = State.kRamping;
    }

    @Override
    public void execute() {
        switch (m_currentState) {
        case kRamping:
            if (m_intake.getPrerollerVelocity() < -8000) {
                m_currentState = State.kOuttake;
                m_intake.setIndexerSpeed(-1.0);
            }
            break;
        case kOuttake:
            break;
        }
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
