// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.IOConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.drive.Swerve;
import wildlib.Toggle;

public class RobotContainer {
    private final CommandXboxController m_controller = new CommandXboxController(IOConstants.controllerPort);

    private static final Swerve m_swerve = Swerve.getInstance();
    // private static final Intake m_intake = Intake.getInstance();
    // private static final Shooter m_shooter = Shooter.getInstance();
    private static final Limelight m_limelight = Limelight.getInstance();
    // Motors not attached yet
    // private static final Climber m_climber = Climber.getInstance();

    private static Toggle m_shootingSpeaker = new Toggle(false);

    public RobotContainer() {
        configureDefaults();
        // configureBindings();

        m_swerve.setDefaultCommand(Commands.run(() -> {
            double forward = MathUtil.applyDeadband(m_controller.getLeftY(), CurrentDriver.getTransDeadband());
            double strafe = MathUtil.applyDeadband(m_controller.getLeftX(), CurrentDriver.getTransDeadband());
            double rotation = MathUtil.applyDeadband(m_controller.getRightX(), CurrentDriver.getRotDeadband());
            double speed = m_controller.getRightTriggerAxis();

            m_swerve.drive(
                forward * speed * (IOConstants.xyInverted ^ CurrentDriver.getXYInverted() ? -1.0 : 1.0),
                strafe * speed * (IOConstants.xyInverted ^ CurrentDriver.getXYInverted() ? -1.0 : 1.0),
                rotation * speed * (IOConstants.rotInverted ^ CurrentDriver.getRotInverted() ? -1.0 : 1.0),
                true, true
            );
        }, m_swerve));
    }

    private void configureDefaults() {
        // m_intake.initDefaultCommand();
        // m_shooter.initDefaultCommand();
    }

    // private void configureBindings() {
    //     m_controller.rightBumper().onTrue(Commands.either(shootAmp(), shootSpeaker(), m_shootingSpeaker));
    //     m_controller.a().onTrue(m_shootingSpeaker.setFalse());
    //     m_controller.y().onTrue(m_shootingSpeaker.setTrue());

    //     NamedCommands.registerCommand("score", shootAmp());
    // }

    // private Command shootAmp() {
    //     return Commands.sequence(
    //         m_shooter.rampAmp(), 
    //         m_intake.advanceAmp().alongWith(m_shooter.waitForShoot())
    //     );
    // }

    // private Command shootSpeaker() {
    //     return Commands.sequence(
    //         m_shooter.rampSpeaker(), 
    //         m_intake.advanceSpeaker().alongWith(m_shooter.waitForShoot())
    //     );
    // }

    public Command getAutonomousCommand() {
        return new PathPlannerAuto("Two-piece Auto");
    }
}
// :)