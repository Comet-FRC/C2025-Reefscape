// CopytopMotor (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.indexer;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

import static edu.wpi.first.units.Units.*;


public class IndexerIOSim implements IndexerIO {
	private final SingleJointedArmSim leftMotor = configureLeftMotor();
	private final SingleJointedArmSim rightMotor = configureRightMotor();

	private static SingleJointedArmSim configureLeftMotor() {
		return new SingleJointedArmSim(
			DCMotor.getNEO(1),
			IndexerConstants.PULLEY_CONVERSION_FACTOR,
			SingleJointedArmSim.estimateMOI(IndexerConstants.LENGTH.in(Meters), IndexerConstants.MASS.in(Kilograms)),
			IndexerConstants.LENGTH.in(Meters),
			0.0,
			Math.PI,
			true,
			0,
			IndexerConstants.ENCODER_DISTANCE_PER_PULSE,
			0
		);
	}
	private static SingleJointedArmSim configureRightMotor() {
		return new SingleJointedArmSim(
			DCMotor.getNEO(1),
			IndexerConstants.PULLEY_CONVERSION_FACTOR,
			SingleJointedArmSim.estimateMOI(IndexerConstants.LENGTH.in(Meters), IndexerConstants.MASS.in(Kilograms)),
			IndexerConstants.LENGTH.in(Meters),
			0.0,
			Math.PI,
			true,
			0,
			IndexerConstants.ENCODER_DISTANCE_PER_PULSE,
			0
		);
	}

	private final PIDController leftPID =
		new PIDController(
			IndexerConstants.LEFT_SIM_kP,
			IndexerConstants.LEFT_SIM_kI,
			IndexerConstants.LEFT_SIM_kD
		);

	private final ArmFeedforward leftFF = new ArmFeedforward(
		IndexerConstants.LEFT_SIM_kS,
		IndexerConstants.LEFT_SIM_kG,
		IndexerConstants.LEFT_SIM_kV,
		IndexerConstants.LEFT_SIM_kA
	);

	private final PIDController rightPID = 
		new PIDController(
			IndexerConstants.RIGHT_SIM_kP,
			IndexerConstants.RIGHT_SIM_kI,
			IndexerConstants.RIGHT_SIM_kD
		);

	private final ArmFeedforward rightFF =
		new ArmFeedforward(
			IndexerConstants.RIGHT_SIM_kS,
			IndexerConstants.RIGHT_SIM_kG,
			IndexerConstants.RIGHT_SIM_kV,
			IndexerConstants.RIGHT_SIM_kA
		);

	@Override
	public void updateInputs(IndexerIOInputs inputs) {
		leftMotor.update(0.02);
		rightMotor.update(0.02);

		runLoopControl();

		inputs.leftPosition = Radians.of(leftMotor.getAngleRads());
		inputs.leftVelocity = RadiansPerSecond.of(leftMotor.getVelocityRadPerSec());
		inputs.leftSupplyCurrent = Amps.of(leftMotor.getCurrentDrawAmps());

		inputs.rightPosition = Radians.of(rightMotor.getAngleRads());
		inputs.rightVelocity = RadiansPerSecond.of(rightMotor.getVelocityRadPerSec());
		inputs.rightSupplyCurrent = Amps.of(rightMotor.getCurrentDrawAmps());
	}

	private void runLoopControl() {
		this.setLeftVoltage(
			Volts.of(
				leftPID.calculate(leftMotor.getAngleRads())
				+
				leftFF.calculate(leftPID.getSetpoint(), 0)
			)
		);
		this.setRightVoltage(
			Volts.of(
				rightPID.calculate(rightMotor.getAngleRads())
				+
				rightFF.calculate(rightPID.getSetpoint(), 0)
			)
		);
	}

	@Override
	public void setLeftPosition(Angle position) {
		leftPID.setSetpoint(position.in(Radians));
	}
	@Override
	public void setRightPosition(Angle position) {
		rightPID.setSetpoint(position.in(Radians));
	}

	// TODO: Make sure these work
	@Override
	public void stopLeft() {
		this.leftPID.setSetpoint(leftMotor.getAngleRads());
	}

	@Override
	public void stopRight() {
		this.rightPID.setSetpoint(rightMotor.getAngleRads());
	}

}
