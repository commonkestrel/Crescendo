package frc.robot.commands.tests;

import java.util.Optional;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Leds;
import frc.robot.subsystems.drive.Swerve;
import wildlib.testing.SystemTestCommand;

public class SwerveTestCommand extends SystemTestCommand {
    private final double RAMP_TIME = 1.0;

    private enum State {
        kForward,
        kBackward,
        kRight,
        kLeft,
        kRotate,
        kContinuous,
    }

    private final Swerve m_swerve;
    private final Leds m_leds;
    private State m_currentState;
    private Timer m_timer = new Timer();

    private boolean m_isFinished = false;
    private Optional<SwerveModuleState> m_forwardFailure = Optional.empty();
    private Optional<SwerveModuleState> m_backwardFailure = Optional.empty();
    private Optional<SwerveModuleState> m_rightFailure = Optional.empty();
    private Optional<SwerveModuleState> m_leftFailure = Optional.empty();
    private Optional<SwerveModuleState> m_rotateFailure = Optional.empty();
    private Optional<SwerveModuleState> m_continuousFailure = Optional.empty();

    public SwerveTestCommand(Swerve swerve, Leds leds) {
        m_swerve = swerve;
        m_leds = leds;
        addRequirements(m_swerve);
    }

    @Override
    public double getTimeout() {
        return 6.5 * RAMP_TIME;
    }

    @Override
    protected void initializeTest() {
        m_currentState = State.kForward;
        m_timer.restart();
    }

    @Override
    protected void executeTest() {
        switch (m_currentState) {
        case kForward:
            if (m_timer.advanceIfElapsed(RAMP_TIME)) {
                
                m_currentState = State.kBackward;
            } else {
                m_swerve.driveRelative(new ChassisSpeeds(DriveConstants.maxTranslationalSpeed, 0.0, 0.0));
            }
            break;
        case kBackward:
            if (m_timer.advanceIfElapsed(RAMP_TIME)) {
                m_currentState = State.kRight;
            } else {
                m_swerve.driveRelative(new ChassisSpeeds(-DriveConstants.maxTranslationalSpeed, 0.0, 0.0));
            }
            break;
        case kRight:
            if (m_timer.advanceIfElapsed(RAMP_TIME)) {
                m_currentState = State.kLeft;
            } else {
                m_swerve.driveRelative(new ChassisSpeeds(0.0, DriveConstants.maxTranslationalSpeed, 0.0));
            }
            break;
        case kLeft:
            if (m_timer.advanceIfElapsed(RAMP_TIME)) {
                m_currentState = State.kRotate;
            } else {
                m_swerve.driveRelative(new ChassisSpeeds(0.0, -DriveConstants.maxTranslationalSpeed, 0.0));
            }
            break;
        case kRotate:
            if (m_timer.advanceIfElapsed(RAMP_TIME)) {
                m_currentState = State.kContinuous;
            } else {
                m_swerve.driveRelative(new ChassisSpeeds(0.0, 0.0, DriveConstants.maxAngularSpeed));
            }
        case kContinuous:
            if (m_timer.advanceIfElapsed(RAMP_TIME)) {
                m_swerve.driveRelative(new ChassisSpeeds(0.0, 0.0, 0.0));
                m_isFinished = true;
                m_leds.flash(Color.kBlue);
            }
            m_swerve.driveRelative(new ChassisSpeeds(Math.cos(m_timer.get() * 2*Math.PI)*DriveConstants.maxTranslationalSpeed, Math.sin(m_timer.get() * 2*Math.PI)*DriveConstants.maxTranslationalSpeed, 0.0));
        }
    }

    @Override
    protected void endTest(boolean interrupted) {
        m_swerve.driveRelative(new ChassisSpeeds(0.0, 0.0, 0.0));
    }

    @Override
    protected boolean isTestFinished() {
        return m_isFinished;
    }

    @Override
    public boolean isTestSuccessful() {
        return m_isFinished
            && m_forwardFailure.isEmpty()
            && m_backwardFailure.isEmpty()
            && m_rightFailure.isEmpty()
            && m_leftFailure.isEmpty()
            && m_rotateFailure.isEmpty()
            && m_continuousFailure.isEmpty();
    }
}
