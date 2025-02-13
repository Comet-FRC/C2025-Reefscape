package frc.robot.subsystems.hoodtake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.util.ArmVisualizer3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

public class Hoodtake extends SubsystemBase {
	private final HoodtakeIO io;
	private final HoodtakeIOInputsAutoLogged inputs = new HoodtakeIOInputsAutoLogged();
	private final ArmVisualizer3d armVisualizer;

	public Hoodtake(HoodtakeIO io) {
		this.io = io;
		this.armVisualizer = new ArmVisualizer3d(getName(), new Translation3d(Units.inchesToMeters(-13), Units.inchesToMeters(13), 0.755615), Rotation2d.fromDegrees(90));

	}

	@Override
	public void periodic() {
		io.updateInputs(inputs);
		Logger.processInputs("Hoodtake", inputs);

		armVisualizer.setArmAngle(inputs.pivotPosition);
		armVisualizer.publish();
	}

	public Command stop() {
		return Commands.run(
				() -> {
					io.stopPivot();
					io.stopWheel();
				},
				this);
	}

	public Command setPosition(Supplier<Angle> position) {
		return Commands.run(() -> io.setPivotPositionSetpoint(position.get()), this);
	}

	public Command setWheelVelocity(Supplier<AngularVelocity> velocity) {
		return Commands.run(() -> io.setWheelVelocitySetpoint(velocity.get()), this);
	}

public Command sysIdRoutinePivot() {
		SysIdRoutine routine = new SysIdRoutine(
			new SysIdRoutine.Config(
				null,
				Volts.of(8.5),
				null,
				(state) -> Logger.recordOutput(
					"SysId/hoodtake-pivot", state.toString()
				)
			),
			new SysIdRoutine.Mechanism(
				io::setPivotVoltage,
				log -> {
					Logger.recordOutput("SysId/hoodtake-pivot/Voltage", inputs.pivotAppliedVolts);
					Logger.recordOutput("SysId/hoodtake-pivot/Position", inputs.pivotPosition);
					Logger.recordOutput("SysId/hoodtake-pivot/Velocity", inputs.pivotVelocity);
					log.motor("hoodtake-pivot")
						.voltage(inputs.pivotAppliedVolts)
						.angularPosition(inputs.pivotPosition)
						.angularVelocity(inputs.pivotVelocity);
				}, 
				this)
		);


		Command routineCommand = new SequentialCommandGroup(
			routine.dynamic(Direction.kReverse),
			Commands.waitSeconds(1),
			routine.dynamic(Direction.kForward),
			Commands.waitSeconds(1),
			routine.quasistatic(Direction.kReverse),
			Commands.waitSeconds(1),
			routine.quasistatic(Direction.kForward)
		);
		

		return routineCommand;
	}
	public Command sysIdRoutineWheel() {
		SysIdRoutine routine = new SysIdRoutine(
			new SysIdRoutine.Config(
				null,
				Volts.of(8.5),
				null,
				(state) -> Logger.recordOutput(
					"SysId/hoodtake-wheel", state.toString()
				)
			),
			new SysIdRoutine.Mechanism(
				io::setWheelVoltage,
				log -> {
					Logger.recordOutput("SysId/hoodtake-wheel/Voltage", inputs.wheelAppliedVolts);
					Logger.recordOutput("SysId/hoodtake-wheel/Velocity", inputs.wheelVelocity);
					Logger.recordOutput("SysId/hoodtake-wheel/Position", inputs.wheelPosition);
					log.motor("hoodtake-wheel")
						.voltage(inputs.wheelAppliedVolts)
						.angularPosition(inputs.wheelPosition)
						.angularVelocity(inputs.wheelVelocity);
				}, 
				this)
		);


		Command routineCommand = new SequentialCommandGroup(
			routine.dynamic(Direction.kForward),
			Commands.waitSeconds(1),
			routine.dynamic(Direction.kReverse),
			Commands.waitSeconds(1),
			routine.quasistatic(Direction.kForward),
			Commands.waitSeconds(1),
			routine.quasistatic(Direction.kReverse)
		);
		return routineCommand;
	}
}
