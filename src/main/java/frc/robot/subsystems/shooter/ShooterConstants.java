package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;

public class ShooterConstants {

 	public static final int TOP_MOTOR_ID = 13;
  	public static final int BOTTOM_MOTOR_ID = 14;

	public static final double WHEEL_CONVERSION_FACTOR = 2 * Math.PI * 2/3; // 2:3 PULLEY RATIO
	public static final Distance WHEEL_RADIUS = Inches.of(3);
	public static final Mass WHEEL_MASS = Pounds.of(0.86);
	public static final double WHEEL_MOMENT_OF_INERTIA = 0.5 * WHEEL_MASS.in(Kilograms) * Math.pow(WHEEL_RADIUS.in(Meters), 2);

	public static final double kP = 0;
	public static final double kI = 0;
	public static final double kD = 0;
	public static final double SIM_kP = 0;
	public static final double SIM_kI = 0;
	public static final double SIM_kD = 0;
 	public static final double kS = 0;
	public static final double kV = 0;
	public static final double kA = 0;
	public static final double SIM_kS = 0;
	public static final double SIM_kV = 0;
	public static final double SIM_kA = 0;

	public static final AngularVelocity ACCEPTABLE_VELOCITY_ERROR = RadiansPerSecond.of(0.1);	
}
