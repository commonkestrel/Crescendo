package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import wildlib.utils.FieldUtils;

public class CrescendoUtils {
    public static final Translation2d DEFAULT_SPEAKER_POSITION = new Translation2d(0.0, 5.4);
    public static final double BLUE_SPEAKER_ARC = Math.PI / 6;
    public static final double RED_SPEAKER_ARC = 5*Math.PI / 6;
    public static final Rotation2d DEFAULT_AMP_ROTATION = new Rotation2d(Math.PI / 2);

    public static Translation2d getAllianceSpeaker() {
        return FieldUtils.correctFieldPosition(DEFAULT_SPEAKER_POSITION);
    }

    public static Rotation2d getAmpOffset() {
        return FieldUtils.correctFieldRotation(DEFAULT_AMP_ROTATION);
    }

    public static Rotation2d correctedSpeakerArc(Translation2d difference) {
        Rotation2d angle = difference.getAngle();
        return clampSpeakerArc(angle);
    }

    public static Rotation2d clampSpeakerArc(Rotation2d angle) {
        if (FieldUtils.red()) {
            double radians = angle.getRadians();
            if (radians < 0.0 && radians > -RED_SPEAKER_ARC) {
                return new Rotation2d(-RED_SPEAKER_ARC);
            } else if (radians > 0.0 && radians < RED_SPEAKER_ARC) {
                return new Rotation2d(RED_SPEAKER_ARC);
            } else {
                return angle;
            }
        } else {
            return new Rotation2d(MathUtil.clamp(angle.getRadians(), -BLUE_SPEAKER_ARC, BLUE_SPEAKER_ARC));
        }
    }
}
