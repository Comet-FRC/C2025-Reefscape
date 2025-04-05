package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

public class DriveConstants {
    public static final double JOYSTICK_DEADBAND_LINEAR = 0.1;
    public static final double JOYSTICK_DEADBAND_ANGULAR = 0.1;

    // These values are also used for pathplanner
    public static final double HEADING_kP = 5.0596;
    public static final double HEADING_kI = 0;
    public static final double HEADING_kD = 0.18657275;

    public static final double HEADING_CORRECTION_kP = 0.5;
    public static final double HEADING_CORRECTION_kI = 0;
    public static final double HEADING_CORRECTION_kD = 0;

    public static final double TRANSLATION_kP = 6;
    public static final double TRANSLATION_kI = 0.0001;
    public static final double TRANSLATION_kD = 0;

    public static final LinearAcceleration MAX_LINEAR_ACCELERATION_PID = MetersPerSecondPerSecond.of(2.75);
    public static final AngularAcceleration MAX_ANGULAR_ACCELERATION_PID = RadiansPerSecondPerSecond.of(Math.PI / 2);

    public static final boolean IS_COSINE_COMPENSATION_ENABLED = true;

    public static final double angularVelocityCoefficient = 0.1;
    public static final boolean IS_ANGULAR_VELOCITY_CORRECTION_ENABLED = true;

    public static final boolean IS_HEADING_CORRECTION_ENABLED = true;
    public static final double HEADING_CORRECTION_DEADBAND = 0.01;

    public static final Alert ALERT_DISCONNECTED_GYRO = new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);
}