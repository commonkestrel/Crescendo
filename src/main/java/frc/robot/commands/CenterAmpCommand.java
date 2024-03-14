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
    }

    private final Swerve m_drive;
    private final Limelight m_limelight;

    private final PIDController m_rotController = new PIDController(AutoConstants.rotKP, AutoConstants.rotKI, AutoConstants.rotKD);
    private final PIDController m_xController = new PIDController(AutoConstants.xKP, AutoConstants.xKI, AutoConstants.xKD);
    private final PIDController m_yController = new PIDController(AutoConstants.yKP, AutoConstants.yKI, AutoConstants.yKD);

    private State m_currentState;


    public CenterAmpCommand(Swerve drive, Limelight limelight) {
        m_drive = drive;
        m_limelight = limelight;

        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        m_limelight.setPipelineIndex(AutoConstants.ampPipeline);
        
        if (m_limelight.getTV()) {
            m_currentState = State.Found;
        } else {
            m_currentState = State.Search;
        }
    
        m_xController.reset();
        m_yController.reset();
        m_rotController.reset();

    }

    @Override
    public void execute() {
        System.out.println(m_limelight.getTV());
        switch (m_currentState) {
        case Search:
            if (m_limelight.getTV()) {
                m_currentState = State.Found;
                m_drive.drive(0.0, 0.0, 0.0, false, false);
                m_xController.setSetpoint(0.0);
                m_yController.setSetpoint(-0.37);
                m_rotController.setSetpoint(0.0);
            } else {
                m_drive.drive(0.0, 0.0, 0.2, false, true);
            }

            break;
        case Found:
            // for (double tx: m_limelight.get) {

            // }
            double[] botpose = m_limelight.getBotPose_TargetSpace();
            double xDistance = botpose[0];
            double yDistance = botpose[2];
            double angle = -botpose[4];

            System.out.printf("Trans Distance: %f", xDistance);
            System.out.printf("TY: %f", m_limelight.getTY());

            double xTranslation = MathUtil.clamp(m_xController.calculate(xDistance), -1.0, 1.0);
            double yTranslation = MathUtil.clamp(m_yController.calculate(yDistance), -1.0, 1.0);
            double rotation = MathUtil.clamp(m_rotController.calculate(angle), -1.0, 1.0);

            m_drive.drive(yTranslation, -xTranslation, rotation, true, false);

            break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.drive(0.0, 0.0, 0.0, false, false);
    }

    @Override
    public boolean isFinished() {
        double[] botpose = m_limelight.getBotPose_TargetSpace();

        return MathUtils.closeEnough(botpose[4], 0.0, 5.0)
            && MathUtils.closeEnough(botpose[0], 0.0, 0.01)
            && MathUtils.closeEnough(botpose[1], -0.37, 0.005)
            && m_limelight.getTV();
    }
}
