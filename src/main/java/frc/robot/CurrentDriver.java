package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriverConstants;

public final class CurrentDriver {
    private enum Driver {
        Default,
        Jett,
        Chris,
        Parker,
    }

    private static final SendableChooser<Driver> m_driver = new SendableChooser<>();

    public static void initDriverStation() {
        m_driver.setDefaultOption("Default", Driver.Default);
        m_driver.addOption("Jett", Driver.Jett);
        m_driver.addOption("Chris", Driver.Chris);
        m_driver.addOption("Parker", Driver.Parker);

        SmartDashboard.putData("Current Driver", m_driver);
    }

    public static boolean getXYInverted() {
        switch (m_driver.getSelected()) {
        case Default:
            return DriverConstants.Default.xyInverted;
        case Jett:
            return DriverConstants.Jett.xyInverted;
        case Chris:
            return DriverConstants.Chris.xyInverted;
        case Parker:
            return DriverConstants.Parker.xyInverted;
        default:
            return DriverConstants.Default.xyInverted;
        }
    }

    public static boolean getRotInverted() {
        switch (m_driver.getSelected()) {
        case Default:
            return DriverConstants.Default.rotInverted;
        case Jett:
            return DriverConstants.Jett.rotInverted;
        case Chris:
            return DriverConstants.Chris.rotInverted;
        case Parker:
            return DriverConstants.Parker.rotInverted;
        default:
            return DriverConstants.Default.rotInverted;
        }
    }

    public static double getTransDeadband() {
        switch (m_driver.getSelected()) {
        case Default:
            return DriverConstants.Default.transDeadband;
        case Jett:
            return DriverConstants.Jett.transDeadband;
        case Chris:
            return DriverConstants.Chris.transDeadband;
        case Parker:
            return DriverConstants.Parker.transDeadband;
        default:
            return DriverConstants.Default.transDeadband;
        }
    }

    public static double getRotDeadband() {
        switch (m_driver.getSelected()) {
        case Default:
            return DriverConstants.Default.rotDeadband;
        case Jett:
            return DriverConstants.Jett.rotDeadband;
        case Chris:
            return DriverConstants.Chris.rotDeadband;
        case Parker:
            return DriverConstants.Parker.rotDeadband;
        default:
            return DriverConstants.Default.rotDeadband;
        }
    }

    public static double getDirSlewRate() {
        switch (m_driver.getSelected()) {
        case Default:
            return DriverConstants.Default.dirSlewRate;
        case Jett:
            return DriverConstants.Jett.dirSlewRate;
        case Chris:
            return DriverConstants.Chris.dirSlewRate;
        case Parker:
            return DriverConstants.Parker.dirSlewRate;
        default:
            return DriverConstants.Default.dirSlewRate;
        }
    }

    // public static double getMagSlewRate() {
    //     switch (m_driver.getSelected()) {
    //     case Default:
    //         return DriverConstants.Default.magSlewRate;
    //     case Jett:
    //         return DriverConstants.Jett.magSlewRate;
    //     case Chris:
    //         return DriverConstants.Chris.magSlewRate;
    //     case Parker:
    //         return DriverConstants.Parker.magSlewRate;
    //     default:
    //         return DriverConstants.Default.magSlewRate;
    //     }
    // }

    // public static double getRotSlewRate() {
    //     switch (m_driver.getSelected()) {
    //     case Default:
    //         return DriverConstants.Default.rotSlewRate;
    //     case Jett:
    //         return DriverConstants.Jett.rotSlewRate;
    //     case Chris:
    //         return DriverConstants.Chris.rotSlewRate;
    //     case Parker:
    //         return DriverConstants.Parker.rotSlewRate;
    //     default:
    //         return DriverConstants.Default.rotSlewRate;
    //     }
    // }
}
