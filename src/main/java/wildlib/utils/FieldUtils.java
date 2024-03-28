package wildlib.utils;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.GameConstants;

public class FieldUtils {
    /**
     * Mirrors a position based on the current alliance
     * 
     * @param rot The initial position from the perspective of the blue Driver Station
     * @return The corrected position
     */
    public static Translation2d correctFieldPosition(Translation2d pos) {
        if (FieldUtils.red()) {
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
        if (FieldUtils.red()) {
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

    public static boolean red() {
        Optional<Alliance> currentAlliance = DriverStation.getAlliance();
        return (currentAlliance.isPresent() && currentAlliance.get() == Alliance.Red);
    }
}
