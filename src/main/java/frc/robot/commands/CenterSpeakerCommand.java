package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CrescendoUtils;
import frc.robot.Constants.AutoConstants;
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
    private final XboxController m_xboxController;

    private final PIDController m_rotController = new PIDController(AutoConstants.rotKP, AutoConstants.rotKI, AutoConstants.rotKD);
    private final PIDController m_xController = new PIDController(AutoConstants.xKP, AutoConstants.xKI, AutoConstants.xKD);
    private final PIDController m_yController = new PIDController(AutoConstants.yKP, AutoConstants.yKI, AutoConstants.yKD);

    private double m_targetRot;
    private double m_targetX;
    private double m_targetY;

    private State m_currentState;


    public CenterSpeakerCommand(Swerve drive, Limelight limelight, Leds leds, XboxController xboxController) {
        m_drive = drive;
        m_limelight = limelight;
        m_leds = leds;
        m_xboxController = xboxController;

        addRequirements(m_drive, m_leds);
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

        m_leds.set(LedState.kFade, Color.kMagenta);
    }

    private void initFound() {
        Pose2d currentPose = m_limelight.getBotPose2d_wpiBlue();
        Translation2d speaker = CrescendoUtils.getAllianceSpeaker();

        Translation2d difference = currentPose.getTranslation().minus(speaker);
        System.out.println(difference.toString());
        System.out.println(DriverStation.getAlliance().toString());

        Rotation2d angle = CrescendoUtils.correctedSpeakerArc(difference);
        setAngle(speaker, angle);
    }

    private void setAngle(Translation2d speaker, Rotation2d angle) {
        m_targetX = AutoConstants.speakerRadius * angle.getCos() + speaker.getX();
        m_targetY = AutoConstants.speakerRadius * angle.getSin() + speaker.getY();
        m_targetRot = new Rotation2d(Math.PI).plus(angle).getDegrees();

        System.out.printf("Target X: %f; Target Y: %f; Target Rot: %f;%n", m_targetX, m_targetY, m_targetRot);
    }

    private boolean isCentered() {
       double[] botpose = m_limelight.getBotPose_wpiBlue();

        return MathUtils.closeEnough(botpose[5], m_targetRot, 5.0)
           && MathUtils.closeEnough(botpose[0], m_targetX, 0.05)
           && MathUtils.closeEnough(botpose[1], m_targetY, 0.05)
            && m_limelight.getTV(); 
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
            int pov = m_xboxController.getPOV();
            System.out.printf("POV: %d%n", pov);
            boolean parallelTranslation = false;
            if (pov == 90 || pov == 270) {

                parallelTranslation = true;

                double distance = (Math.PI/6) / 20;
                if (pov == 270) {
                    distance = -distance;
                }
                Optional<Alliance> currentAlliance = DriverStation.getAlliance();
                if (currentAlliance.isPresent() || currentAlliance.get() == Alliance.Red) {
                    distance = -distance;
                }

                Rotation2d angle = CrescendoUtils.clampSpeakerArc(FieldUtils.correctFieldRotation(Rotation2d.fromDegrees(m_targetRot)).unaryMinus().plus(Rotation2d.fromRadians(distance)));
                Translation2d speaker = CrescendoUtils.getAllianceSpeaker();

                setAngle(speaker, angle);
            }
        if (!parallelTranslation || !isCentered()) {
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
        } else {
            m_leds.set(LedState.kSolid, Color.kPlum);
            }
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
        return false;
        // double[] botpose = m_limelight.getBotPose_wpiBlue();

        // return MathUtils.closeEnough(botpose[5], m_targetRot, 5.0)
        //     && MathUtils.closeEnough(botpose[0], m_targetX, 0.05)
        //     && MathUtils.closeEnough(botpose[1], m_targetY, 0.05)
        //     && m_limelight.getTV();
    }
}