package wildlib.utils;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.GameConstants;

public class FieldUtils {
    public static final Translation2d DEFAULT_SPEAKER_POSITION = new Translation2d(0.0, 5.4);
    public static final Rotation2d DEFAULT_AMP_ROTATION = new Rotation2d(Math.PI / 2);

    public static Translation2d getAllianceSpeaker() {
        return correctFieldPosition(DEFAULT_SPEAKER_POSITION);
    }

    public static Rotation2d getAmpOffset() {
        return correctFieldRotation(DEFAULT_AMP_ROTATION);
    }

    /**
     * Mirrors a position based on the current alliance
     * 
     * @param rot The initial position from the perspective of the blue Driver Station
     * @return The corrected position
     */
    public static Translation2d correctFieldPosition(Translation2d pos) {
        Optional<Alliance> currentAlliance = DriverStation.getAlliance();

        if (currentAlliance.isPresent() && currentAlliance.get() == Alliance.Red) {
            return flipFieldPosition(pos);
        } else {
            return pos;
        }
    }

    /**
     * Mirrors a rotation based on the current alliance
     * 
     * @param rot The initial rotation from the perspective of the blue Driver Station
     * @return The corrected rotation
     */
    public static Rotation2d correctFieldRotation(Rotation2d rot) {
        Optional<Alliance> currentAlliance = DriverStation.getAlliance();

        if (currentAlliance.isPresent() && currentAlliance.get() == Alliance.Red) {
            return flipFieldRotation(rot);
        } else {
            return rot;
        }
    }

    /**
     * Mirrors a position to the other side of the field
     * 
     * @param pos The initial position
     * @return The mirrored position
     */
    public static Translation2d flipFieldPosition(Translation2d pos) {
        return new Translation2d(GameConstants.fieldLength - pos.getX(), pos.getY());
    }

    /**
     * Flips a rotation to the opposite driver station.
     * 
     * @param rot The initial rotation
     * @return The flipped rotation
    */
    public static Rotation2d flipFieldRotation(Rotation2d rot) {
        return new Rotation2d(Math.PI).minus(rot);
    }
}
