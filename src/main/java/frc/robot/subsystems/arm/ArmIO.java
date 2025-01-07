package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.measure.Angle;
import edu.wpi.first.units.measure.measure.AngularVelocity;
import edu.wpi.first.units.measure.measure.Current;
import edu.wpi.first.units.measure.measure.Temperature;
import edu.wpi.first.units.measure.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  public static class ArmIOInputs {
    public boolean topMotorConnected = false;
    public boolean botMotorConnected = false;

    public Angle topPosition = Radians.of(0.0);
    public AngularVelocity topVelocity = RadiansPerSecond.of(0.0);
    public Voltage topAppliedVoltage = Volts.of(0.0);
    public Current topSupplyCurrent = Amps.of(0.0);
    public Current topTorqueCurrent = Amps.of(0.0);
    public Temperature topTemperature = Celsius.of(0.0);

    public Angle botPosition = Radians.of(0.0);
    public AngularVelocity botVelocity = RadiansPerSecond.of(0.0);
    public Voltage botAppliedVoltage = Volts.of(0.0);
    public Current botSupplyCurrent = Amps.of(0.0);
    public Current botTorqueCurrent = Amps.of(0.0);
    public Temperature botTemperature = Celsius.of(0.0);
  }

  default void updateInputs(ArmIOInputs inputs) {}

  /** Run both motors at voltage */
  default void runVolts(Voltage topVolts, Voltage bottomVolts) {}

  /** Stop both flywheels */
  default void stop() {}

  /** Run motor velocities in rpm */
  default void runVelocity(
      AngularVelocity topVelocity,
      AngularVelocity botVelocity,
      double leftFeedforward,
      double rightFeedforward) {}

  /** Config PID values for both motors */
  default void setPID(double kP, double kI, double kD) {}

  /** Config FF values for both motors */
  default void setFF(double kS, double kV, double kA) {}

  /** Run left flywheels at voltage */
  default void runCharacterizationTop(double input) {}

  /** Run right flywheels at voltage */
  default void runCharacterizationBot(double input) {}
}
