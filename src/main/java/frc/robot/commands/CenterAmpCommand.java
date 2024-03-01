package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.drive.Swerve;
import wildlib.utils.MathUtils;

public class CenterAmpCommand extends Command {
    private enum State {
        Search,
        Found,
        Forward,
    }

    private final Swerve m_drive;
    private final Limelight m_limelight;

    private final PIDController m_rotController = new PIDController(AutoConstants.rotKP, AutoConstants.rotKI, AutoConstants.rotKD);
    private final PIDController m_transController = new PIDController(AutoConstants.transKP, AutoConstants.transKI, AutoConstants.transKD);

    private Command m_driveCommand;
    private State m_currentState;


    public CenterAmpCommand(Swerve drive, Limelight limelight) {
        m_drive = drive;
        m_limelight = limelight;
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        m_driveCommand = null;
        m_limelight.setPipelineIndex(AutoConstants.ampPipeline);
        if (m_limelight.getTV()) {
            m_currentState = State.Found;
        } else {
            m_currentState = State.Search;
        }
    }

    @Override
    public void execute() {
        System.out.println(m_limelight.getTV());
        switch (m_currentState) {
        case Search:
            if (m_limelight.getTV()) {
                m_currentState = State.Found;
                m_drive.drive(0.0, 0.0, 0.0, false, false);
                m_transController.setSetpoint(0.0);
                m_rotController.setSetpoint(0.0);
            } else {
                m_drive.drive(0.0, 0.0, 0.2, false, true);
            }

            break;
        case Found:
            double averageDistance = m_limelight.getBotPose()[6];
            double angle = m_limelight.getTX() - m_drive.getAmpOffset() + m_drive.getHeading();
            double transDistance = averageDistance * Math.sin(Units.degreesToRadians(angle));

            if (
                MathUtils.closeEnough(m_limelight.getTX(), 0.0, 5.0)
                && MathUtils.closeEnough(transDistance, 0.0, 0.25)
            ) {
                m_currentState = State.Forward;
                m_drive.drive(0.0, 0.0, 0.0, false, false);
                Pose2d targetPose = m_drive.getPose();
                targetPose.transformBy(
                    new Transform2d(new Translation2d(averageDistance, 0.0),
                    new Rotation2d())
                );
                
                m_driveCommand = AutoBuilder.pathfindToPose(
                    targetPose,
                    new PathConstraints(DriveConstants.maxAngularSpeed, DriveConstants.maxAcceleration, DriveConstants.maxAngularSpeed, DriveConstants.maxAngularAccel)
                );

                break;
            }

            double translation = m_transController.calculate(transDistance);
            double rotation = m_rotController.calculate(m_limelight.getTX());

            m_drive.drive(translation, 0.0, rotation, true, true);

            break;
        case Forward:
            
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (m_driveCommand != null) {
            m_driveCommand.end(interrupted);
        }
    }

    @Override
    public boolean isFinished() {
        return m_currentState == State.Forward && m_driveCommand.isFinished();
    }
}
