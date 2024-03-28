package wildlib.testing;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class SystemTestCommand extends Command {
    private static final double DEFAULT_TIMEOUT = 5.0;
    private final Timer m_timer = new Timer();

    protected double getTimeout() { return DEFAULT_TIMEOUT; };

    protected void initializeTest() {}
    protected void executeTest() {}
    protected void endTest(boolean interrupted) {}
    protected boolean isTestFinished() { return true; }

    public boolean isTestSuccessful() { return true; }
    public String getErrorMessage() { return ""; }

    @Override
    public void initialize() {
        if (DriverStation.isTest()) {
            initializeTest();
            m_timer.restart();
        }
    }

    @Override
    public void execute() {
        if (DriverStation.isTest()) {
            executeTest();
        }
    }

    @Override
    public void end(boolean interrupted) {
        if(DriverStation.isTest()) {
            endTest(!isTestFinished());
        }
    }

    @Override
    public boolean isFinished() {
        return !DriverStation.isTest() || m_timer.hasElapsed(getTimeout()) || isTestFinished();
    }
}
