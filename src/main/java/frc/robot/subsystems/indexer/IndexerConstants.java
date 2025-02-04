package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Distance;

public final class IndexerConstants {
	public static final int LEFT_MOTOR_ID = 17;
	public static final int RIGHT_MOTOR_ID = 18;

	public static final double PULLEY_CONVERSION_FACTOR = 2 * Math.PI;

	public static final double LEFT_kP = 0;
	public static final double LEFT_kI = 0;
	public static final double LEFT_kD = 0;
	public static final double LEFT_SIM_kP = 0;
	public static final double LEFT_SIM_kI = 0;
	public static final double LEFT_SIM_kD = 0;
	public static final double LEFT_kS = 0;
	public static final double LEFT_kG = 0;
	public static final double LEFT_kV = 0;
	public static final double LEFT_kA = 0;
	public static final double LEFT_SIM_kS = 0;
	public static final double LEFT_SIM_kG = 0;
	public static final double LEFT_SIM_kV = 0;
	public static final double LEFT_SIM_kA = 0;

	public static final double RIGHT_kP = 0;
	public static final double RIGHT_kI = 0;
	public static final double RIGHT_kD = 0;
	public static final double RIGHT_SIM_kP = 0;
	public static final double RIGHT_SIM_kI = 0;
	public static final double RIGHT_SIM_kD = 0;
	public static final double RIGHT_kS = 0;
	public static final double RIGHT_kG = 0;
	public static final double RIGHT_kV = 0;
	public static final double RIGHT_kA = 0;
	public static final double RIGHT_SIM_kS = 0;
	public static final double RIGHT_SIM_kG = 0;
	public static final double RIGHT_SIM_kV = 0;
	public static final double RIGHT_SIM_kA = 0;

	public static final Distance LENGTH = Inches.of(18);
	public static final Mass MASS = Pounds.of(5.2);
	public static final double ENCODER_DISTANCE_PER_PULSE = 2.0 * Math.PI / 4096;
}
