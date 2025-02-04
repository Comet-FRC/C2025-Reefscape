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
		public AngularVelocity leftVelocity = RadiansPerSecond.of(0);
		public Voltage leftAppliedVoltage = Volts.of(0);
		public Current leftSupplyCurrent = Amps.of(0);
		public Temperature leftTemperature = Celsius.of(0);

		public Angle rightPosition = Radians.of(0);
		public AngularVelocity rightVelocity = RadiansPerSecond.of(0);
		public Voltage rightAppliedVoltage = Volts.of(0);
		public Current rightSupplyCurrent = Amps.of(0);
		public Temperature rightTemperature = Celsius.of(0);
	}

	public default void updateInputs(IndexerIOInputs inputs) {}

	public default void stop() {
		stopLeft();
		stopRight();	
	}
	public default void stopLeft() {}
	public default void stopRight() {}

	public default void setLeftVelocity(AngularVelocity velocity) {}
	public default void setLeftVoltage(Voltage volts) {}
	public default void setLeftPosition(Angle position) {}
	public default void setRightVelocity(AngularVelocity velocity) {}
	public default void setRightVoltage(Voltage volts) {}
	public default void setRightPosition(Angle position) {}

	public default void setLeftPID(double kP, double kI, double kD) {}
	public default void setLeftFF(double kS, double kV, double kA) {}
	public default void setRightPID(double kP, double kI, double kD) {}
	public default void setRightFF(double kS, double kV, double kA) {}
	
	public default void runCharacterizationLeft(double input) {}
	public default void runCharacterizationRight(double input) {}

}
