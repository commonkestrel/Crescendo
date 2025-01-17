// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
// import frc.robot.commands.tests.ClimberTestCommand;
import frc.robot.commands.tests.IntakeTestCommand;
import frc.robot.commands.tests.ShooterTestCommand;
import frc.robot.commands.tests.SwerveTestCommand;
// import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Leds;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.drive.Swerve;
import wildlib.testing.SystemTest;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
    private RobotContainer m_robotContainer;

    @Override
    public void robotInit() {
        // Initialize the robot container
        // This is where the majority of the actual robot logic is found
        m_robotContainer = new RobotContainer();

        // Register system tests
        SystemTest.registerTest("Swerve", new SwerveTestCommand(Swerve.getInstance(), Leds.getInstance()));
        SystemTest.registerTest("Intake", new IntakeTestCommand(Intake.getInstance()));
        SystemTest.registerTest("Shooter", new ShooterTestCommand(Shooter.getInstance(), Leds.getInstance()));

        SystemTest.loadTests();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        SystemTest.run();
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}
}
