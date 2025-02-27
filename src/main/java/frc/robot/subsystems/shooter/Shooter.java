package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class Shooter extends SubsystemBase {
	public final ShooterIO io;
	public final ShooterIOInputsAutoLogged inputs;
	private final RangeTable RANGE_TABLE = new RangeTable();

	public Shooter(ShooterIO ShooterIO) {
		this.io = ShooterIO;
		this.inputs = new ShooterIOInputsAutoLogged();
	}

	@Override
	public void periodic() {
		io.updateInputs(inputs);
		Logger.processInputs("Shooter", inputs);
	}


	public boolean readyToShoot(){
		if (inputs.topWheelDesiredVelocity.minus(inputs.topWheelVelocity).abs(RPM) > ShooterConstants.ACCEPTABLE_VELOCITY_ERROR.in(RPM))
			return false;
		if (inputs.bottomWheelDesiredVelocity.minus(inputs.bottomWheelVelocity).abs(RPM) > ShooterConstants.ACCEPTABLE_VELOCITY_ERROR.in(RPM))
			return false;
		return true;
	}

	public Command stop() {
		return Commands.run(() -> io.setWheelVelocitySetpoint(RPM.of(0), RPM.of(0)));
	}

	public Command setFlywheelVelocitiesFromDistance(Supplier<Distance> distance) {
		return setFlywheelVelocities(RANGE_TABLE.get(distance::get));
	}

	public Command setFlywheelVelocities(Supplier<ShooterSpeed> shooterSpeeds) {
		return this.setFlywheelVelocities(
			() -> shooterSpeeds.get().topMotorSpeed,
			() -> shooterSpeeds.get().botMotorSpeed
		);
	}

	public Command setFlywheelVelocities(Supplier<AngularVelocity> topSpeed, Supplier<AngularVelocity> botSpeed) {
		return Commands.runOnce(() -> {
				io.setWheelVelocitySetpoint(topSpeed.get(), botSpeed.get());
		});
	}

	public Command topSysId() {
		SysIdRoutine routine = new SysIdRoutine(
			new SysIdRoutine.Config(
				null,
				Volts.of(4),
				null,
				(state) -> Logger.recordOutput(
					"SysId/shooter-wheel", state.toString()
				)
			),
			new SysIdRoutine.Mechanism(
				io::setTopVoltage,
				log -> {
					Logger.recordOutput("SysId/shooter-wheel/Voltage", inputs.topWheelAppliedVoltage);
					Logger.recordOutput("SysId/shooter-wheel/Velocity", inputs.topWheelVelocity);
					Logger.recordOutput("SysId/shooter-wheel/Position", inputs.topWheelPosition);
					log.motor("shooter-wheel")
						.voltage(inputs.topWheelAppliedVoltage)
						.angularPosition(inputs.topWheelPosition)
						.angularVelocity(inputs.topWheelVelocity);
				}, 
				this)
		);


		Command routineCommand = new SequentialCommandGroup(
			routine.dynamic(Direction.kForward),
			Commands.waitSeconds(3),
			routine.dynamic(Direction.kReverse),
			Commands.waitSeconds(3),
			routine.quasistatic(Direction.kForward),
			Commands.waitSeconds(3),
			routine.quasistatic(Direction.kReverse)
		);
		return routineCommand;
	}

	public Command bottomSysId() {
		SysIdRoutine routine = new SysIdRoutine(
			new SysIdRoutine.Config(
				null,
				Volts.of(4),
				null,
				(state) -> Logger.recordOutput(
					"SysId/bottom-shooter-wheel", state.toString()
				)
			),
			new SysIdRoutine.Mechanism(
				io::setBottomVoltage,
				log -> {
					Logger.recordOutput("SysId/bottom-shooter-wheel/Voltage", inputs.bottomWheelAppliedVoltage);
					Logger.recordOutput("SysId/bottom-shooter-wheel/Velocity", inputs.bottomWheelVelocity);
					Logger.recordOutput("SysId/bottom-shooter-wheel/Position", inputs.bottomWheelPosition);
					log.motor("bottom-shooter-wheel")
						.voltage(inputs.bottomWheelAppliedVoltage)
						.angularPosition(inputs.bottomWheelPosition)
						.angularVelocity(inputs.bottomWheelVelocity);
				}, 
				this)
		);


		Command routineCommand = new SequentialCommandGroup(
			routine.dynamic(Direction.kForward),
			Commands.waitSeconds(3),
			routine.dynamic(Direction.kReverse),
			Commands.waitSeconds(3),
			routine.quasistatic(Direction.kForward),
			Commands.waitSeconds(3),
			routine.quasistatic(Direction.kReverse)
		);
		return routineCommand;
	}

	public Command setWheelVoltages(Supplier<Voltage> volts) {
		return Commands.run(
			() -> {
				io.setTopVoltage(volts.get());
				io.setBottomVoltage(volts.get());
			}
		, this);
	}

	public Command setTopVoltage(Supplier<Voltage> volts) {
		return Commands.run(() -> io.setTopVoltage(volts.get()), this);
	}

	public Command setBottomVoltage(Supplier<Voltage> volts) {
		return Commands.run(() -> io.setBottomVoltage(volts.get()), this);
	}
}
