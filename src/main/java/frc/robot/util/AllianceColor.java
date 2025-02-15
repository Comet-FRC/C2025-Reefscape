package frc.robot.util;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * A utility class for determining the alliance color of the robot.
 */
public final class AllianceColor {

    /**
     * @return true if the robot is on the red alliance
     */
    public static final boolean isRed() {
        Optional<Alliance> ally = DriverStation.getAlliance();
        return ally.isPresent() && ally.get() == Alliance.Red;
    }
}
