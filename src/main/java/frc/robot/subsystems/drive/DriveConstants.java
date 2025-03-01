package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

public class DriveConstants {
    public static final LinearVelocity MINUMUM_VELOCITY = MetersPerSecond.of(0.3);
    public static final AngularVelocity MINUMUM_ANGULAR_VELOCITY = RadiansPerSecond.of(0.01);

    // These values are also used for pathplanner
    public static final double HEADING_kP = 5.0596;
    public static final double HEADING_kI = 0;
    public static final double HEADING_kD = 0.18657275;

    public static final double TRANSLATION_kP = 2.5;
    public static final double TRANSLATION_kI = 0;
    public static final double TRANSLATION_kD = 0;

    public static final double angularVelocityCoefficient = 0.0;
    public static final boolean angularVelocityCorrection = true;

    public static final boolean headingCorrection = true;
    public static final double HEADING_CORRECTION_DEADBAND = 0.01;

    public static final Alert ALERT_DISCONNECTED_GYRO = new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);
}