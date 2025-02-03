package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.units.measure.*;

public interface ShooterIO {
	@AutoLog
	public static class ShooterIOInputs {
		public AngularVelocity topVelocity = RadiansPerSecond.of(0);
		public AngularVelocity topDesiredVelocity = RadiansPerSecond.of(0);
		public Voltage topAppliedVoltage = Volts.of(0);
		public Current topSupplyCurrent = Amps.of(0);
		public Temperature topTemperature = Celsius.of(0);

		public AngularVelocity bottomVelocity = RadiansPerSecond.of(0);
		public AngularVelocity bottomDesiredVelocity = RadiansPerSecond.of(0);
		public Voltage bottomAppliedVoltage = Volts.of(0);
		public Current bottomSupplyCurrent = Amps.of(0);
		public Temperature bottomTemperature = Celsius.of(0);
	}

	public default void updateInputs(ShooterIOInputs inputs) {}

	default void setVoltage(Voltage topVoltage, Voltage bottomVoltage) {}

	default void stop() {}

	default void setAngularVelocity(ShooterSpeed shooterSpeed) {}

	default void setPID(double kP, double kI, double kD) {}
	default void setFF(double kS, double kV, double kA) {}

	default void runCharacterizationbottomMotor(Voltage input) {}
	default void runCharacterizationtopMotor(Voltage input) {}
}
