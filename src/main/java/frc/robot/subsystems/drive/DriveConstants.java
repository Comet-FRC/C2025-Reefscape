package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;

public class DriveConstants {
    public static final LinearVelocity MINUMUM_VELOCITY = MetersPerSecond.of(0.1);
    public static final AngularVelocity MINUMUM_ANGULAR_VELOCITY = RadiansPerSecond.of(0.01);

    public static final double HEADING_kP = 1;
    public static final double HEADING_kI = 0;
    public static final double HEADING_kD = 0;
}