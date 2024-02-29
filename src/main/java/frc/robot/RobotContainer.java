// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ClimberReleaseCommand;
import frc.robot.Constants.IOConstants;
import frc.robot.commands.ClimberRetractCommand;
import frc.robot.commands.IntakeIdleCommand;
import frc.robot.commands.IntakeSourceCommand;
import frc.robot.commands.RampAmpCommand;
import frc.robot.commands.RampSpeakerCommand;
import frc.robot.commands.ShootAmpCommand;
import frc.robot.commands.ShootSpeakerCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.drive.Swerve;
import wildlib.Toggle;

public class RobotContainer {
    private final CommandXboxController m_driveController = new CommandXboxController(IOConstants.driveControllerPort);
    private final CommandXboxController m_mechController = new CommandXboxController(IOConstants.mechControllerPort);

    private static final Swerve m_swerve = Swerve.getInstance();
    private static final Intake m_intake = Intake.getInstance();
    private static final Shooter m_shooter = Shooter.getInstance();
    private static final Limelight m_limelight = Limelight.getInstance();
    // Motors not attached yet
    private static final Climber m_climber = Climber.getInstance();

    private static Toggle m_shootingSpeaker = new Toggle(false);

    public RobotContainer() {
        configureSmartDashboard();
        configureDefaults();
        configureBindings();

        m_swerve.setDefaultCommand(Commands.run(() -> {
            double forward = MathUtil.applyDeadband(m_driveController.getLeftY(), CurrentDriver.getTransDeadband());
            double strafe = MathUtil.applyDeadband(m_driveController.getLeftX(), CurrentDriver.getTransDeadband());
            double rotation = MathUtil.applyDeadband(m_driveController.getRightX(), CurrentDriver.getRotDeadband());
            double speed = m_driveController.getRightTriggerAxis();

            m_swerve.drive(
                forward * speed * (IOConstants.xyInverted ^ CurrentDriver.getXYInverted() ? -1.0 : 1.0),
                strafe * speed * (IOConstants.xyInverted ^ CurrentDriver.getXYInverted() ? -1.0 : 1.0),
                rotation * speed * (IOConstants.rotInverted ^ CurrentDriver.getRotInverted() ? -1.0 : 1.0),
                true, true, m_driveController.getLeftTriggerAxis()>0
            );
        }, m_swerve));
    }

    private void configureSmartDashboard() {
        CurrentDriver.initDriverStation();
    }

    private void configureDefaults() {
        m_intake.setDefaultCommand(new IntakeIdleCommand(m_intake));
        m_shooter.initDefaultCommand();
    }

    private void configureBindings() {
        m_mechController.rightBumper().whileTrue(Commands.either(shootSpeaker(), shootAmp(), m_shootingSpeaker));
        m_mechController.leftBumper().whileTrue(Commands.either(new RampSpeakerCommand(m_shooter), new RampAmpCommand(m_shooter), m_shootingSpeaker));

        m_mechController.a().onTrue(m_shootingSpeaker.setFalse());
        m_mechController.y().onTrue(m_shootingSpeaker.setTrue());
        m_mechController.b().whileTrue(new IntakeSourceCommand(m_intake, m_shooter));

        m_mechController.povDown().whileTrue(new ClimberRetractCommand(m_climber));
        m_mechController.povUp().whileTrue(new ClimberReleaseCommand(m_climber));

        m_driveController.x().onTrue(Commands.run(m_swerve::crossWheels));
        m_driveController.start().onTrue(Commands.runOnce(m_swerve::zeroHeading, m_swerve));

        NamedCommands.registerCommand("scoreAmp", shootAmp());
        NamedCommands.registerCommand("scoreSpeaker", shootSpeaker());
    }

    private Command shootAmp() {
        return new ShootAmpCommand(m_intake, m_shooter);
    }

    private Command shootSpeaker() {
        return new ShootSpeakerCommand(m_intake, m_shooter);
    }

    public Command getAutonomousCommand() {
        return new ParallelCommandGroup(
            new PathPlannerAuto("Two-piece Auto"),
            new ClimberRetractCommand(m_climber)
        );
    }
}
// :)