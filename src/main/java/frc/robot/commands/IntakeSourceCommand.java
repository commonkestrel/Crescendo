package frc.robot.commands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Leds;
import frc.robot.subsystems.Shooter;

public class IntakeSourceCommand extends Command {
    enum State {
        NotPassed,
        Passing,
        Passed,
        Equilibrium
    }
    private final Intake m_intake;
    private final Shooter m_shooter;
    private final Leds m_leds;

    private State m_currentState;

    public IntakeSourceCommand(Intake intake, Shooter shooter, Leds leds) {
        m_intake = intake; 
        m_shooter = shooter;
        m_leds = leds;
        addRequirements(m_intake, m_shooter);
    }

    @Override
    public void initialize() {
        m_intake.setPrerollerSpeed(0.0);
        m_intake.setIndexerSpeed(-0.3);
        // m_shooter.setTargetVelocity(ShooterConstants.intakeTarget);
        m_shooter.setSpeed(-0.25);
        m_currentState = State.NotPassed;
    }

    @Override
    public void execute() {
        switch (m_currentState) {
        case NotPassed:
            if (m_intake.noteDetected()) {
                m_leds.flash(Color.kOrchid);
                m_shooter.setSpeed(0.0);
                m_currentState = State.Passing;
            }
            break;
        case Passing:
            if (!m_intake.noteDetected()) {
                m_intake.setIndexerSpeed(0.1);
                m_currentState = State.Passed;
            }
            break;
        case Passed:
            if (m_intake.noteDetected()) {
                m_leds.flash(Color.kGreen);
                m_intake.stop();
                m_currentState = State.Equilibrium;
            }
            break;
        case Equilibrium:
            break;
    }
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.stop();
    }

    @Override
    public boolean isFinished() {
        // Finished if we've seen a Note and it has passed the sensor completely
        return m_currentState == State.Equilibrium;
    }
}
