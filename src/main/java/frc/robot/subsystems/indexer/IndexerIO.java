package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.*;

public interface IndexerIO {
	@AutoLog
	public static class IndexerIOInputs {
		public Angle leftPosition = Radians.of(0);
		public Angle leftPositionSetpoint = Radians.of(0);
		public AngularVelocity leftVelocity = RadiansPerSecond.of(0);
		public Voltage leftAppliedVoltage = Volts.of(0);
		public Current leftSupplyCurrent = Amps.of(0);
		public Temperature leftTemperature = Celsius.of(0);

		public Angle rightPosition = Radians.of(0);
		public Angle rightPositionSetpoint = Radians.of(0);
		public AngularVelocity rightVelocity = RadiansPerSecond.of(0);
		public Voltage rightAppliedVoltage = Volts.of(0);
		public Current rightSupplyCurrent = Amps.of(0);
		public Temperature rightTemperature = Celsius.of(0);
	}

	public default void updateInputs(IndexerIOInputs inputs) {}

	/** Sets the desired position of the left indexer. */
	public default void setLeftPositionSetpoint(Angle position) {}
	/** Sets the desired velocity of the left indexer. */
	public default void setLeftVelocitySetpoint(AngularVelocity velocity) {}
	public default void setLeftVoltage(Voltage volts) {}
	
	/** Sets the desired position of the right indexer. */
	public default void setRightPositionSetpoint(Angle position) {}
	/** Sets the desired velocity of the right indexer. */
	public default void setRightVelocitySetpoint(AngularVelocity velocity) {}
	public default void setRightVoltage(Voltage volts) {}

	public default void stopLeft() {}
	public default void stopRight() {}
	public default void stop() {
		stopLeft();
		stopRight();	
	}	
}
