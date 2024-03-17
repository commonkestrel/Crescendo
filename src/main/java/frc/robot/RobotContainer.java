// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.PPLibTelemetry;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.CenterAmpCommand;
import frc.robot.commands.CenterCommand;
import frc.robot.commands.CenterSpeakerCommand;
import frc.robot.commands.ClimberReleaseCommand;
import frc.robot.Constants.IOConstants;
import frc.robot.commands.ClimberRetractCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakeIdleCommand;
import frc.robot.commands.IntakeSourceCommand;
import frc.robot.commands.OuttakeCommand;
import frc.robot.commands.RampAmpCommand;
import frc.robot.commands.RampSpeakerCommand;
import frc.robot.commands.ResetHeading;
import frc.robot.commands.ShootAmpCommand;
import frc.robot.commands.ShootSpeakerCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Leds;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Leds.LedState;
import frc.robot.subsystems.drive.Swerve;
import wildlib.Toggle;

public class RobotContainer {
    private final CommandXboxController m_driveController = new CommandXboxController(IOConstants.driveControllerPort);
    private final CommandXboxController m_mechController = new CommandXboxController(IOConstants.mechControllerPort);

    private static final Swerve m_swerve = Swerve.getInstance();
    private static final Intake m_intake = Intake.getInstance();
    private static final Shooter m_shooter = Shooter.getInstance();
    private static final Climber m_climber = Climber.getInstance();
    private static final Limelight m_limelight = Limelight.getInstance();
    private static final Leds m_leds = Leds.getInstance();

    private static Toggle m_shootingSpeaker = new Toggle(false);
    private static 
    Trigger m_limelightTarget;

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
                true, false
            );
        }, m_swerve));
    }

    private void configureSmartDashboard() {
        CurrentDriver.initDriverStation();
    }

    private void configureDefaults() {
        m_intake.setDefaultCommand(new IntakeIdleCommand(m_intake, m_leds));
        m_shooter.initDefaultCommand();
    }

    private void configureBindings() {
        m_mechController.rightBumper().whileTrue(Commands.either(shootSpeaker(), shootAmp(), m_shootingSpeaker));
        m_mechController.leftBumper().whileTrue(Commands.either(new RampSpeakerCommand(m_shooter), new RampAmpCommand(m_shooter), m_shootingSpeaker));
        m_mechController.x().whileTrue(new OuttakeCommand(m_intake));

        m_mechController.a().onTrue(m_shootingSpeaker.setFalse());
        m_mechController.y().onTrue(m_shootingSpeaker.setTrue());
        m_mechController.b().whileTrue(new IntakeSourceCommand(m_intake, m_shooter));
        // m_mechController.x().whileTrue(new IntakeCommand(m_intake));

        m_mechController.povDown().whileTrue(new ClimberRetractCommand(m_climber, m_leds));
        m_mechController.povUp().whileTrue(new ClimberReleaseCommand(m_climber));
        m_mechController.start().onTrue(new InstantCommand(m_intake::toggleOverride));

        m_driveController.back().onTrue(Commands.runOnce(m_swerve::capSpeed, m_swerve));

        m_driveController.x().whileTrue(Commands.run(m_swerve::crossWheels, m_swerve));
        m_driveController.start().onTrue(Commands.runOnce(m_swerve::zeroHeading, m_swerve));
        m_driveController.b().whileTrue(new CenterAmpCommand(m_swerve, m_limelight, m_leds).andThen(Commands.print("centered")));
        m_driveController.a().onTrue(Commands.runOnce(() -> m_leds.set(LedState.kRainbow, Color.kBlack)));

        m_driveController.leftBumper()
            .whileTrue(Commands.race(new CenterAmpCommand(m_swerve, m_limelight, m_leds), new RampSpeakerCommand(m_shooter))
            .andThen(new ShootSpeakerCommand(m_intake, m_shooter)));

        m_driveController.rightBumper().whileTrue(new CenterSpeakerCommand(m_swerve, m_limelight, m_leds));
        m_driveController.a().whileTrue(new CenterCommand(m_swerve, m_limelight, m_leds));

        // Initialize limelight
        m_limelightTarget = new Trigger(m_limelight::getTV);
        m_limelightTarget.onTrue(Commands.runOnce(() -> m_leds.set(LedState.kSolid, Color.kBlue)));
        m_limelightTarget.onFalse(Commands.runOnce(() -> m_leds.set(LedState.kFade, Color.kRed)));
        m_leds.set(m_limelight.getTV() ? LedState.kSolid : LedState.kFade, m_limelight.getTV() ? Color.kBlue : Color.kRed);

        NamedCommands.registerCommand("scoreAmp", shootAmp());
        NamedCommands.registerCommand("scoreSpeaker", shootSpeaker());
        NamedCommands.registerCommand("rampAmp", new RampAmpCommand(m_shooter));
        NamedCommands.registerCommand("rampSpeaker", new RampSpeakerCommand(m_shooter));
    }

    private Command shootAmp() {
        return new ShootAmpCommand(m_intake, m_shooter);
    }

    private Command shootSpeaker() {
        return new ShootSpeakerCommand(m_intake, m_shooter);
    }

    public Command getAutonomousCommand() {
        return null;
        // return new ClimberRetractCommand(m_climber);
    }

    public Command getLimelightAuto() {
        double offset = m_swerve.getAmpOffset();

        return Commands.sequence(
            new ResetHeading(m_swerve, offset),
            Commands.parallel(
                new ClimberRetractCommand(m_climber, m_leds)
            )
        );
    }

    public Command getStartToAmp() {
        return AutoBuilder.buildAuto("One Note");
    }

    public Command getDriveBack() {
        Command driveBack =  new RunCommand(
            () -> m_swerve.drive(0.0, 0.2, 0.0, false, true), m_swerve
        ).withTimeout(1.0)
        .andThen(
            new InstantCommand(() -> m_swerve.drive(0.0, 0.0, 0.0, false, false), m_swerve)
        );
        
        driveBack.addRequirements(m_swerve);
        return driveBack;
    }
}
// :)