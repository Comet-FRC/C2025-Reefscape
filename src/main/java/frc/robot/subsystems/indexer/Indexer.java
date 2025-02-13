package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.util.ArmVisualizer3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
	private final IndexerIO io;
	private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();
	private final ArmVisualizer3d leftArmVisualizer3d;
	private final ArmVisualizer3d rightArmVisualizer;


	public Indexer(IndexerIO io) {
		this.io = io;
		this.leftArmVisualizer3d = new ArmVisualizer3d("leftIndexer", new Translation3d(0,0,0), Rotation2d.fromDegrees(0));
		this.rightArmVisualizer = new ArmVisualizer3d("rightIndexer", new Translation3d(0,0,0), Rotation2d.fromDegrees(0));
	}

	@Override
	public void periodic() {
		io.updateInputs(inputs);
		Logger.processInputs("Indexer", inputs);

		leftArmVisualizer3d.setArmAngle(inputs.leftPosition);
		leftArmVisualizer3d.publish();

		rightArmVisualizer.setArmAngle(inputs.rightPosition);
		rightArmVisualizer.publish();
	}

	private Command stop() {
		return
			Commands.run(
				() -> {
					io.stop();
				},
			this);
	}

	public Command setLeftPosition(Supplier<Angle> position) {
		return Commands.run(() -> io.setLeftPositionSetpoint(position.get()), this);
	}

	public Command setRightPosition(Supplier<Angle> position) {
		return Commands.run(() -> io.setRightPositionSetpoint(position.get()), this);
	}

	public Command sysIdRoutineLeft() {
		SysIdRoutine routine = new SysIdRoutine(
			new SysIdRoutine.Config(
				null,
				Volts.of(6),
				null,
				(state) -> Logger.recordOutput(
					"SysId/indexer-left", state.toString()
				)
			),
			new SysIdRoutine.Mechanism(
				io::setLeftVoltage,
				log -> {
					Logger.recordOutput("SysId/indexer-left/Voltage", inputs.leftAppliedVoltage);
					Logger.recordOutput("SysId/indexer-left/Position", inputs.leftPosition);
					Logger.recordOutput("SysId/indexer-left/Velocity", inputs.leftVelocity);
					log.motor("indexer-left")
						.voltage(inputs.leftAppliedVoltage)
						.angularPosition(inputs.leftPosition)
						.angularVelocity(inputs.leftVelocity);
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

	public Command sysIdRoutineRight() {
		SysIdRoutine routine = new SysIdRoutine(
			new SysIdRoutine.Config(
				null,
				Volts.of(8.5),
				null,
				(state) -> Logger.recordOutput(
					"SysId/indexer-right", state.toString()
				)
			),
			new SysIdRoutine.Mechanism(
				io::setLeftVoltage,
				log -> {
					Logger.recordOutput("SysId/indexer-right/Voltage", inputs.rightAppliedVoltage);
					Logger.recordOutput("SysId/indexer-right/Position", inputs.rightPosition);
					Logger.recordOutput("SysId/indexer-right/Velocity", inputs.rightVelocity);
					log.motor("indexer-right")
						.voltage(inputs.rightAppliedVoltage)
						.angularPosition(inputs.rightPosition)
						.angularVelocity(inputs.rightVelocity);
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
