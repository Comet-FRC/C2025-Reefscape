package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;

public class ShooterConstants {

 	public static final int TOP_MOTOR_ID = 13;
  	public static final int BOTTOM_MOTOR_ID = 14;

	public static final double TOP_WHEEL_CONVERSION_FACTOR = 2 * Math.PI * (15.0/18.0); // 2:3 PULLEY RATIO
	public static final double BOTTOM_WHEEL_CONVERSION_FACTOR = 2 * Math.PI * (9.0/18.0);
	public static final Distance WHEEL_RADIUS = Inches.of(3);
	public static final Mass WHEEL_MASS = Pounds.of(0.86);
	public static final double WHEEL_MOMENT_OF_INERTIA = 0.5 * WHEEL_MASS.in(Kilograms) * Math.pow(WHEEL_RADIUS.in(Meters), 2);

	public static final double TOP_WHEEL_kP = 0.01;
	public static final double TOP_WHEEL_kI = 0.000004;
	public static final double TOP_WHEEL_kD = 10;
	public static final double TOP_WHEEL_kS = 0.048874;
	public static final double TOP_WHEEL_kV = 0.04;
	public static final double TOP_WHEEL_kA = 0.011556;
	
	public static final double BOT_WHEEL_kP = 0.009;
	public static final double BOT_WHEEL_kI = 0.000006;
	public static final double BOT_WHEEL_kD = 100;
	public static final double BOT_WHEEL_kS = 0.048874;
	public static final double BOT_WHEEL_kV = 0.041276;
	public static final double BOT_WHEEL_kA = 0.011556;
	
	public static final double WHEEL_SIM_kP = 0.65;
	public static final double WHEEL_SIM_kI = 0.01;
	public static final double WHEEL_SIM_kD = 0;
	public static final double WHEEL_SIM_kS = 0;
	public static final double WHEEL_SIM_kV = 0.1;
	public static final double WHEEL_SIM_kA = 0.00094517;

	public static final AngularVelocity ACCEPTABLE_VELOCITY_ERROR = RPM.of(10);	
}
