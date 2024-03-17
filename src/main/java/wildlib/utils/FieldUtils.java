package wildlib.utils;

import java.util.Optional;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class FieldUtils {
    public static Translation2d getAllianceSpeaker() {
        Optional<Alliance> currentAlliance = DriverStation.getAlliance();
        if (currentAlliance.isPresent()) {
            switch (currentAlliance.get()) {
                case Blue:
                    return new Translation2d(0.2, 5.4);
                case Red:
                    return new Translation2d(16.0, 5.4);
                default:
                    return new Translation2d(0.0, 0.0);
            }
        } else {
            return new Translation2d(0, 0);
        }
    }


}
