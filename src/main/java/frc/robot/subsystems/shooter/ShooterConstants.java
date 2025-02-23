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

	public static final double TOP_WHEEL_kP = 0.26136;
	public static final double TOP_WHEEL_kI = 0;
	public static final double TOP_WHEEL_kD = 0;
	public static final double TOP_WHEEL_kS = 0.040598;
	public static final double TOP_WHEEL_kV = 0.2604;
	public static final double TOP_WHEEL_kA = 1.1668;
	
	public static final double BOT_WHEEL_kP = 0.26136;
	public static final double BOT_WHEEL_kI = 0;
	public static final double BOT_WHEEK_kD = 0;
	public static final double BOT_WHEEL_kS = 0.040598;
	public static final double BOT_WHEEL_kV = 0.2604;
	public static final double BOT_WHEEL_kA = 1.1668;
	
	public static final double WHEEL_SIM_kP = 0.013657;
	public static final double WHEEL_SIM_kI = 0;
	public static final double WHEEL_SIM_kD = 0;
	public static final double WHEEL_SIM_kS = -0.0041375;
	public static final double WHEEL_SIM_kV = 0.083116;
	public static final double WHEEL_SIM_kA = 0.00094517;

	public static final AngularVelocity ACCEPTABLE_VELOCITY_ERROR = RadiansPerSecond.of(0.1);	
}
