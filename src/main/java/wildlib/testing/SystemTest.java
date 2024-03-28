package wildlib.testing;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class SystemTest {
    private static final ShuffleboardTab m_tab = Shuffleboard.getTab("System Test");
    private static final GenericEntry m_manual = m_tab.add("Manual", true)
        .withWidget(BuiltInWidgets.kToggleSwitch).getEntry();
    private static boolean m_manualPrevious = true;

    private static SequentialCommandGroup m_autoCommand;

    private static final HashMap<String, SystemTestCommand> m_tests = new HashMap<>();

    public static void registerTest(String name, SystemTestCommand test) {
        m_tests.put(name, test);
    }

    public static void loadTests() {
        for (Map.Entry<String, SystemTestCommand> entry : m_tests.entrySet()) {
            m_tab.add(entry.getKey(), entry.getValue().beforeStarting(new InstantCommand(() -> {
                m_manual.setBoolean(true);
                cancelAllExcept(entry.getKey());
            })));
        }
    }

    public static void run() {
        if (DriverStation.isTest()) {
            if (m_manual.getBoolean(true) != m_manualPrevious) {
                matchManual();
            }
        }
    }

    private static void matchManual() {
        cancelAll();

        if (!m_manual.getBoolean(true)) {
            enableAuto();
        }
    }

    private static void enableAuto() {
        m_autoCommand = new SequentialCommandGroup();
        for (Map.Entry<String, SystemTestCommand> entry : m_tests.entrySet()) {
            SystemTestCommand value = entry.getValue();
            String key = entry.getKey();

            m_autoCommand.addCommands(value, new InstantCommand(() -> {
                if (value.isTestSuccessful()) {
                    System.out.printf("Test `%s` successful%n", key);
                } else {
                    DriverStation.reportError(String.format("Test `%s` failed with error message: %s%n", key, value.getErrorMessage()), false);
                }
            }));
        }

        m_autoCommand.schedule();
    }

    private static void cancelAllExcept(String except) {
        if (m_autoCommand != null) {
            m_autoCommand.end(true);
            m_autoCommand.cancel();
        }

        for (Map.Entry<String, SystemTestCommand> entry : m_tests.entrySet()) {
            if (entry.getKey() != except) {
                entry.getValue().end(entry.getValue().isScheduled());
                entry.getValue().cancel();
            }
        }
    }

    private static void cancelAll() {
        cancelAllExcept("");
    }
}
