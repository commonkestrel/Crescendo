package frc.robot.commands.tests;

import frc.robot.subsystems.drive.Swerve;
import wildlib.testing.SystemTestCommand;

public class SwerveTestCommand extends SystemTestCommand {
    private final Swerve m_swerve;

    public SwerveTestCommand(Swerve swerve) {
        m_swerve = swerve;
        addRequirements(m_swerve);
    }

    @Override
    protected void initializeTest() {

    }

    @Override
    protected void executeTest() {

    }

    @Override
    protected void endTest(boolean interrupted) {
        m_swerve.drive(0.0, 0.0, 0.0, false, false);
    }

    @Override
    protected boolean isTestFinished() {
        return true;
    }

    @Override
    public boolean isTestSuccessful() {
        return isTestFinished();
    }
}
