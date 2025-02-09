package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;

public class ShooterConstants {

 	public static final int TOP_MOTOR_ID = 13;
  	public static final int BOTTOM_MOTOR_ID = 14;

	public static final double WHEEL_CONVERSION_FACTOR = 2 * Math.PI * 2/3; // 2:3 PULLEY RATIO
	public static final Distance WHEEL_RADIUS = Inches.of(3);
	public static final Mass WHEEL_MASS = Pounds.of(0.86);
	public static final double WHEEL_MOMENT_OF_INERTIA = 0.5 * WHEEL_MASS.in(Kilograms) * Math.pow(WHEEL_RADIUS.in(Meters), 2);

	public static final double topWheelkP = 0;
	public static final double topWheelkI = 0;
	public static final double topWheelkD = 0;
	public static final double topWheelSIM_kP = 0;
	public static final double topWheelSIM_kI = 0;
	public static final double topWheelSIM_kD = 0;
 	public static final double topWheelkS = 0;
	public static final double topWheelkV = 0;
	public static final double topWheelkA = 0;
	public static final double topWheelSIM_kS = 0;
	public static final double topWheelSIM_kV = 0;
	public static final double topWheelSIM_kA = 0;

	public static final double bottomWheelkP = 0;
	public static final double bottomWheelkI = 0;
	public static final double bottomWheelkD = 0;
	public static final double bottomWheelSIM_kP = 0;
	public static final double bottomWheelSIM_kI = 0;
	public static final double bottomWheelSIM_kD = 0;
 	public static final double bottomWheelkS = 0;
	public static final double bottomWheelkV = 0;
	public static final double bottomWheelkA = 0;
	public static final double bottomWheelSIM_kS = 0;
	public static final double bottomWheelSIM_kV = 0;
	public static final double bottomWheelSIM_kA = 0;

	public static final AngularVelocity ACCEPTABLE_VELOCITY_ERROR = RadiansPerSecond.of(0.1);	
}
