package frc.robot.subsystems.hoodtake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.ArmVisualizer3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

public class Hoodtake extends SubsystemBase {
	private final HoodtakeIO io;
	private final HoodtakeIOInputsAutoLogged inputs = new HoodtakeIOInputsAutoLogged();
	private final ArmVisualizer3d armVisualizer;

	public Hoodtake(HoodtakeIO io) {
		this.io = io;
		this.armVisualizer = new ArmVisualizer3d(getName(), new Translation3d(0,0,0), Rotation2d.fromDegrees(0));

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
}
