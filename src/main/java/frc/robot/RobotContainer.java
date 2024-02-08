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
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.drive.Swerve;

public class RobotContainer {
    private final CommandXboxController m_controller = new CommandXboxController(IOConstants.controllerPort);

    private static final Swerve m_swerve = Swerve.getInstance();
    private static final Intake m_intake = Intake.getInstance();
    private static final Limelight m_limelight = Limelight.getInstance();

    public RobotContainer() {
        configureBindings();

        m_swerve.setDefaultCommand(Commands.run(() -> {
            double forward = MathUtil.applyDeadband(m_controller.getLeftY(), IOConstants.transDeadband);
            double strafe = MathUtil.applyDeadband(m_controller.getLeftX(), IOConstants.transDeadband);
            double rotation = MathUtil.applyDeadband(m_controller.getRightX(), IOConstants.rotDeadband);
            double speed = m_controller.getRightTriggerAxis();

            m_swerve.drive(
                forward * speed * (IOConstants.xyInverted ? -1.0 : 1.0),
                strafe * speed * (IOConstants.xyInverted ? -1.0 : 1.0),
                rotation * speed * (IOConstants.rotInverted ? -1.0 : 1.0),
                true, true
            );
        }, m_swerve));

        m_intake.setDefaultCommand(Commands.run(() -> {
            if (m_intake.noteDetected()) {
                m_intake.stop();
            } else {
                m_intake.setSpeed(0.8);
            }
        }, m_intake));
    }

    private void configureBindings() {
        m_controller.rightBumper().onTrue(m_intake.advance());

        NamedCommands.registerCommand("score", m_intake.waitForNote().andThen(m_intake.advance()));
    }

    public Command getAutonomousCommand() {
        return new PathPlannerAuto("Two-piece Auto");
    }
}
