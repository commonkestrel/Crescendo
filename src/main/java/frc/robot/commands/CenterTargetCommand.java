package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Leds;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Leds.LedState;
import frc.robot.subsystems.drive.Swerve;
import wildlib.utils.MathUtils;

public class CenterTargetCommand extends Command {
    private enum State {
        Search,
        Found,
    }

    private final Swerve m_drive;
    private final Limelight m_limelight;
    private final Leds m_leds;
    private final Double m_yDistance;

    private final PIDController m_rotController = new PIDController(AutoConstants.rotKP, AutoConstants.rotKI, AutoConstants.rotKD);
    private final PIDController m_xController = new PIDController(AutoConstants.xKP, AutoConstants.xKI, AutoConstants.xKD);
    private final PIDController m_yController = new PIDController(AutoConstants.yKP, AutoConstants.yKI, AutoConstants.yKD);

    private State m_currentState;


    public CenterTargetCommand(Swerve drive, Limelight limelight, Leds leds, Double yDistance) {
        m_drive = drive;
        m_limelight = limelight;
        m_leds = leds;
        m_yDistance = yDistance;

        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        m_limelight.setPipelineIndex(AutoConstants.ampPipeline);
        
        if (m_limelight.getTV()) {
           initFound();
        } else {
            m_currentState = State.Search;
        }
    
        m_xController.reset();
        m_yController.reset();
        m_rotController.reset();

        m_rotController.setIZone(Double.POSITIVE_INFINITY);

        m_leds.set(LedState.kFade, Color.kMagenta);

    }

    @Override
    public void execute() {
        System.out.println(m_limelight.getTV());
        switch (m_currentState) {
        case Search:
            if (m_limelight.getTV()) {
                initFound();
            } else {
                m_drive.drive(0.0, 0.0, 0.2, false, true);
            }

            break;
        case Found:
            System.out.printf("Y PID Setpoint: %f; ", m_yController.getSetpoint());

            double[] botpose = m_limelight.getBotPose_TargetSpace();
            double xDistance = botpose[0];
            double yDistance = botpose[2];
            double angle = -botpose[4];

            System.out.printf("Gyro offset: %f; Gyro angle: %f%n", m_drive.getOffset(), m_drive.getAngle());
            System.out.printf("4: %s; 0: %s; 1: %s%n", Boolean.toString(MathUtils.closeEnough(botpose[4], 0.0, 5.0)), Boolean.toString(MathUtils.closeEnough(botpose[0], 0.0, 0.01)), Boolean.toString(MathUtils.closeEnough(botpose[1], -0.39, 0.02)));

            System.out.printf("Y Distance: %f; ", yDistance);

            double xTranslation = MathUtil.clamp(m_xController.calculate(xDistance), -1.0, 1.0);
            double rotation = MathUtil.clamp(m_rotController.calculate(angle), -1.0, 1.0);
            System.out.printf("Angle: %f; AnglePID: %f%n", angle, rotation);
            System.out.printf("X Distance: %f; XPID: %f%n", xDistance, xTranslation);
            SmartDashboard.putNumber("Angle offset", angle);

            double yTranslation; 
            if (m_yDistance != null) {
                yTranslation = MathUtil.clamp(m_yController.calculate(yDistance), -1.0, 1.0);
                System.out.printf("Y Distance: %f; Y PID Output: %f%n", yDistance, yTranslation);
            } else {
                yTranslation = 0.0;
            }

            m_drive.drive(yTranslation, -xTranslation, rotation, true, false);
            break;
        }
    }

    public void initFound() {
        m_currentState = State.Found;
        m_drive.drive(0.0, 0.0, 0.0, false, false);
        m_drive.setOffset(m_drive.getHeading() + m_limelight.getBotPose_TargetSpace()[4]);

        m_xController.setSetpoint(0.0);
        if (m_yDistance != null) m_yController.setSetpoint(m_yDistance);
        m_rotController.setSetpoint(0.0 /* FieldUtils.getAmpOffset().getDegrees() */);
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.drive(0.0, 0.0, 0.0, false, false);
        m_drive.setOffset(0.0);
        m_leds.set(m_limelight.getTV() ? LedState.kSolid : LedState.kFade, m_limelight.getTV() ? Color.kBlue : Color.kRed);
    }

    @Override
    public boolean isFinished() {
        double[] botpose = m_limelight.getBotPose_TargetSpace();

        return MathUtils.closeEnough(botpose[4], 0.0, 0.01)
            && MathUtils.closeEnough(botpose[0], 0.0, 0.03)
            && (m_yDistance == null || MathUtils.closeEnough(botpose[2], m_yDistance, 0.01))
            && m_limelight.getTV();
    }
}
