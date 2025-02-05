package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.util.ArmVisualizer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
	private final IntakeIO io;
	private final IntakeIOInputsAutoLogged inputs;
	private final ArmVisualizer armVisualizer;

	public Intake(IntakeIO io) {
		this.io = io;
		this.inputs = new IntakeIOInputsAutoLogged();
		this.armVisualizer = new ArmVisualizer(getName(), IntakeConstants.LENGTH);
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
		return Commands.runOnce(() -> io.setPivotPosition(position.get()), this);
	}

	public Command setWheelVelocity(Supplier<AngularVelocity> velocity) {
		return Commands.runOnce(() -> io.setWheelVelocity(velocity.get()), this);
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
}
