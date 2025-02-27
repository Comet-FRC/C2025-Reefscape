package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.units.measure.*;

public interface ShooterIO {
	@AutoLog
	public static class ShooterIOInputs {
		public AngularVelocity topWheelVelocity = RadiansPerSecond.of(0);
		public AngularVelocity topWheelDesiredVelocity = RadiansPerSecond.of(0);
		public Angle topWheelPosition = Radians.of(0);
		public Voltage topWheelAppliedVoltage = Volts.of(0);
		public Current topWheelSupplyCurrent = Amps.of(0);
		public Temperature topTemperature = Celsius.of(0);

		public AngularVelocity bottomWheelVelocity = RadiansPerSecond.of(0);
		public AngularVelocity bottomWheelDesiredVelocity = RadiansPerSecond.of(0);
		public Angle bottomWheelPosition = Radians.of(0);
		public Voltage bottomWheelAppliedVoltage = Volts.of(0);
		public Current bottomWheelSupplyCurrent = Amps.of(0);
		public Temperature bottomWheelTemperature = Celsius.of(0);
	}

	public default void updateInputs(ShooterIOInputs inputs) {}
	public default void stop() {}
	public default void setWheelVelocitySetpoint(AngularVelocity topVelocity, AngularVelocity bottomVelocity) {}
	public default void setTopVoltage(Voltage volts) {}
	public default void setBottomVoltage(Voltage volts) {}
}
