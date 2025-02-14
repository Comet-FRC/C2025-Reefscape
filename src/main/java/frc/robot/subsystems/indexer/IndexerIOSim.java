// CopytopMotor (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.indexer;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
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
			Math.PI/2,
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
			Math.PI/2,
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

	private boolean leftVoltageMode = false;
	private boolean rightVoltageMode = false;
	private final MutVoltage leftAppliedVoltage = Volts.mutable(0);
	private final MutVoltage rightAppliedVoltage = Volts.mutable(0);

	@Override
	public void updateInputs(IndexerIOInputs inputs) {
		leftMotor.update(0.02);
		rightMotor.update(0.02);

		runLoopControl();

		inputs.leftPosition = Radians.of(leftMotor.getAngleRads());
		inputs.leftPositionSetpoint = Radians.of(leftPID.getSetpoint());
		inputs.leftVelocity = RadiansPerSecond.of(leftMotor.getVelocityRadPerSec());
		inputs.leftAppliedVoltage = this.leftAppliedVoltage.copy();
		inputs.leftSupplyCurrent = Amps.of(leftMotor.getCurrentDrawAmps());

		inputs.rightPosition = Radians.of(rightMotor.getAngleRads());
		inputs.rightPositionSetpoint = Radians.of(rightPID.getSetpoint());
		inputs.rightVelocity = RadiansPerSecond.of(rightMotor.getVelocityRadPerSec());
		inputs.rightAppliedVoltage = this.rightAppliedVoltage.copy();
		inputs.rightSupplyCurrent = Amps.of(rightMotor.getCurrentDrawAmps());
	}

	private void runLoopControl() {
		if (!leftVoltageMode) {
			double pidOutput = leftPID.calculate(leftMotor.getAngleRads());
			double ffOutput = leftFF.calculate(leftPID.getSetpoint(), 0);
			double totalOutput = pidOutput + ffOutput;
			this.leftAppliedVoltage.mut_replace(totalOutput, Volts);
			this.leftMotor.setInputVoltage(totalOutput);
		}

		if (!rightVoltageMode) {
			this.rightMotor.setInputVoltage(
				rightPID.calculate(rightMotor.getAngleRads())
				+
				rightFF.calculate(rightPID.getSetpoint(), 0)
			);
		}
	}

	@Override
	public void setLeftPositionSetpoint(Angle position) {
		this.leftVoltageMode = false;
		leftPID.setSetpoint(position.in(Radians));
	}
	@Override
	public void setRightPositionSetpoint(Angle position) {
		this.rightVoltageMode = false;
		rightPID.setSetpoint(position.in(Radians));
	}

	@Override
	public void setLeftVoltage(Voltage volts) {
		this.leftVoltageMode = true;
		this.leftMotor.setInputVoltage(volts.in(Volts));
		this.leftAppliedVoltage.mut_replace(volts);
	}

	@Override
	public void setRightVoltage(Voltage volts) {
		this.rightVoltageMode = true;
		this.rightMotor.setInputVoltage(volts.in(Volts));
		this.rightAppliedVoltage.mut_replace(volts);
	}

	// TODO: Make sure these work
	@Override
	public void stopLeft() {
		this.setLeftVoltage(Volts.of(0));
	}

	@Override
	public void stopRight() {
		this.setRightVoltage(Volts.of(0));
	}

}
