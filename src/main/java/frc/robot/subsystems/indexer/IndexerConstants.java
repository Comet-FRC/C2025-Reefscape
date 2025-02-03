package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Distance;

public final class IndexerConstants {
	public static final int WHEEL_MOTOR_ID = 17;
	public static final int PIVOT_MOTOR_ID = 18;

	public static final double WHEEL_CONVERSION_FACTOR = 2 * Math.PI;
	public static final double PIVOT_CONVERSION_FACTOR = 2 * Math.PI * 0.5; // 1:2 PULLEY RATIO

	public static final double PIVOT_kP = 0;
	public static final double PIVOT_kI = 0;
	public static final double PIVOT_kD = 0;
	public static final double PIVOT_SIM_kP = 0;
	public static final double PIVOT_SIM_kI = 0;
	public static final double PIVOT_SIM_kD = 0;
	public static final double PIVOT_kS = 0;
	public static final double PIVOT_kG = 0;
	public static final double PIVOT_kV = 0;
	public static final double PIVOT_kA = 0;
	public static final double PIVOT_SIM_kS = 0;
	public static final double PIVOT_SIM_kG = 0;
	public static final double PIVOT_SIM_kV = 0;
	public static final double PIVOT_SIM_kA = 0;

	public static final double WHEEL_kP = 0;
	public static final double WHEEL_kI = 0;
	public static final double WHEEL_kD = 0;
	public static final double WHEEL_SIM_kP = 0;
	public static final double WHEEL_SIM_kI = 0;
	public static final double WHEEL_SIM_kD = 0;
	public static final double WHEEL_kS = 0;
	public static final double WHEEL_kV = 0;
	public static final double WHEEL_kA = 0;
	public static final double WHEEL_SIM_kS = 0;
	public static final double WHEEL_SIM_kV = 0;
	public static final double WHEEL_SIM_kA = 0;

	public static final Mass WHEEL_MASS = Pounds.of(0.035);
	public static final Distance WHEEL_RADIUS = Inches.of(1);
	public static final double WHEEL_MOI = 0.5 * WHEEL_MASS.in(Kilograms) * Math.pow(WHEEL_RADIUS.in(Meters), 2); // 1/2MR^2

	public static final Distance LENGTH = Inches.of(18);
	public static final Mass MASS = Pounds.of(5.2);
	public static final double PIVOT_ENCODER_DISTANCE_PER_PULSE = 2.0 * Math.PI / 4096;
}
