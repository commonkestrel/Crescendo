package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Leds;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Leds.LedState;
import frc.robot.subsystems.drive.Swerve;
import wildlib.utils.FieldUtils;
import wildlib.utils.MathUtils;

public class CenterSpeakerCommand extends Command {
    private enum State {
        Search,
        Found,
    }

    private final Swerve m_drive;
    private final Limelight m_limelight;
    private final Leds m_leds;

    private final PIDController m_rotController = new PIDController(AutoConstants.rotKP, AutoConstants.rotKI, AutoConstants.rotKD);
    private final PIDController m_xController = new PIDController(AutoConstants.xKP, AutoConstants.xKI, AutoConstants.xKD);
    private final PIDController m_yController = new PIDController(AutoConstants.yKP, AutoConstants.yKI, AutoConstants.yKD);

    private double m_targetRot;
    private double m_targetX;
    private double m_targetY;

    private State m_currentState;


    public CenterSpeakerCommand(Swerve drive, Limelight limelight, Leds leds) {
        m_drive = drive;
        m_limelight = limelight;
        m_leds = leds;

        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        m_limelight.setPipelineIndex(AutoConstants.ampPipeline);
        
        if (m_limelight.getTV()) {
            initFound();
            m_currentState = State.Found;
        } else {
            m_currentState = State.Search;
        }
    
        m_xController.reset();
        m_yController.reset();
        m_rotController.reset();
        m_rotController.enableContinuousInput(-180, 180);

        m_leds.set(LedState.kFade, Color.kOrange);
    }

    private void initFound() {
        Pose2d currentPose = m_limelight.getBotPose2d_wpiBlue();
        Translation2d speaker = FieldUtils.getAllianceSpeaker();

        Translation2d difference = currentPose.getTranslation().minus(speaker);

        double angle = MathUtil.clamp(Math.atan(difference.getY() / difference.getX()), -Math.PI/6, Math.PI/6);
        m_targetX = AutoConstants.speakerRadius * Math.cos(angle) + speaker.getX();
        m_targetY = AutoConstants.speakerRadius * Math.sin(angle) + speaker.getY();
        m_targetRot = 180 + Units.radiansToDegrees(angle);

        System.out.printf("Target X: %f; Target Y: %f; Target Rot: %f;%n", m_targetX, m_targetY, m_targetRot);
    }

    @Override
    public void execute() {
        System.out.println(m_limelight.getTV());
        switch (m_currentState) {
        case Search:
            if (m_limelight.getTV()) {
                initFound();

                m_currentState = State.Found;
                m_drive.drive(0.0, 0.0, 0.0, false, false);
                m_xController.setSetpoint(m_targetX);
                m_yController.setSetpoint(m_targetY);
                m_rotController.setSetpoint(m_targetRot);
            } else {
                m_drive.drive(0.0, 0.0, 0.2, false, true);
            }

            break;
        case Found:
            m_xController.setSetpoint(m_targetX);
            m_yController.setSetpoint(m_targetY);
            m_rotController.setSetpoint(m_targetRot);    

            double[] botpose = m_limelight.getBotPose_wpiBlue();
            double xDistance = botpose[0];
            double yDistance = botpose[1];
            double angle = botpose[5];

            double xTranslation = MathUtil.clamp(m_xController.calculate(xDistance), -1.0, 1.0);
            double yTranslation = MathUtil.clamp(m_yController.calculate(yDistance), -1.0, 1.0);
            System.out.printf("Y Distance: %f; Y PID Output: %f%n", yDistance, yTranslation);

            double rotation = MathUtil.clamp(m_rotController.calculate(angle), -1.0, 1.0);
            System.out.printf("Angle: %f; AnglePID: %f%n", angle, rotation);
            System.out.printf("X Distance: %f; XPID: %f%n", xDistance, xTranslation);

            m_drive.drive(xTranslation, yTranslation, rotation, true, false);
            break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.drive(0.0, 0.0, 0.0, false, false);
        m_leds.set(m_limelight.getTV() ? LedState.kSolid : LedState.kFade, m_limelight.getTV() ? Color.kBlue : Color.kRed);
    }

    @Override
    public boolean isFinished() {
        double[] botpose = m_limelight.getBotPose_TargetSpace();

        return MathUtils.closeEnough(botpose[5], m_targetRot, 5.0)
            && MathUtils.closeEnough(botpose[0], m_targetX, 0.03)
            && MathUtils.closeEnough(botpose[1], m_targetY, 0.03)
            && m_limelight.getTV();
    }
}