package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

public class DriveConstants {
    public static final LinearVelocity MINUMUM_VELOCITY = MetersPerSecond.of(0.1);
    public static final AngularVelocity MINUMUM_ANGULAR_VELOCITY = RadiansPerSecond.of(0.01);

    public static final double HEADING_kP = 0.1;
    public static final double HEADING_kI = 0;
    public static final double HEADING_kD = 0.01;



    public static final Alert ALERT_DISCONNECTED_GYRO = new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);
}