package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.util.ArmVisualizer;
import frc.robot.util.ArmVisualizer3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
	private final IntakeIO io;
	private final IntakeIOInputsAutoLogged inputs;
	private final ArmVisualizer3d armVisualizer;

	public Intake(IntakeIO io) {
		this.io = io;
		this.inputs = new IntakeIOInputsAutoLogged();
		this.armVisualizer = new ArmVisualizer3d(getName(), new Translation3d(0,0.378-0.044,0.184), Rotation2d.fromDegrees(0));
	}

	@Override
	public void periodic() {
		io.updateInputs(inputs);
		Logger.processInputs("Intake", inputs);

		armVisualizer.setArmAngle(inputs.pivotPosition);
		armVisualizer.publish();
	}

	private Command stop() {
		return
			Commands.run(
				() -> {
					io.stopPivot();
					io.stopWheel();
				},
			this);
	}

	public Command setPosition(Supplier<Angle> position) {
		return Commands.runOnce(() -> io.setPivotPositionSetpoint(position.get()), this);
	}

	public Command setWheelVelocity(Supplier<AngularVelocity> velocity) {
		return Commands.runOnce(() -> io.setWheelVelocitySetpoint(velocity.get()), this);
	}

	public Command setPivotVoltage(Supplier<Voltage> volts) {
		return Commands.runOnce(() -> io.setPivotVoltage(volts.get()), this);
	}

	public Command sysIdRoutinePivot() {
		SysIdRoutine routine = new SysIdRoutine(
			new SysIdRoutine.Config(
				null,
				Volts.of(8.5),
				null,
				(state) -> Logger.recordOutput(
					"SysId/intake-pivot", state.toString()
				)
			),
			new SysIdRoutine.Mechanism(
				io::setPivotVoltage,
				log -> {
					Logger.recordOutput("SysId/intake-pivot/Voltage", inputs.pivotAppliedVoltage);
					Logger.recordOutput("SysId/intake-pivot/Position", inputs.pivotPosition);
					Logger.recordOutput("SysId/intake-pivot/Velocity", inputs.pivotVelocity);
					log.motor("intake-pivot")
						.voltage(inputs.pivotAppliedVoltage)
						.angularPosition(inputs.pivotPosition)
						.angularVelocity(inputs.pivotVelocity);
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
	public Command sysIdRoutineWheel() {
		SysIdRoutine routine = new SysIdRoutine(
			new SysIdRoutine.Config(
				null,
				Volts.of(8.5),
				null,
				(state) -> Logger.recordOutput(
					"SysId/intake-wheel", state.toString()
				)
			),
			new SysIdRoutine.Mechanism(
				io::setWheelVoltage,
				log -> {
					Logger.recordOutput("SysId/intake-wheel/Voltage", inputs.wheelAppliedVoltage);
					Logger.recordOutput("SysId/intake-wheel/Velocity", inputs.wheelVelocity);
					Logger.recordOutput("SysId/intake-wheel/Position", inputs.wheelPosition);
					log.motor("intake-wheel")
						.voltage(inputs.wheelAppliedVoltage)
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
