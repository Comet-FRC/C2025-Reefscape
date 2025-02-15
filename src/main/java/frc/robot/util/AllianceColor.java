package frc.robot.util;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public final class AllianceColor {
    public static final boolean isRed() {
        Optional<Alliance> ally = DriverStation.getAlliance();
        return ally.isPresent() && ally.get() == Alliance.Red;
    }
}
