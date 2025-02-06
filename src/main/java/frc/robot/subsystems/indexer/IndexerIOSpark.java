// CopytopMotor (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Celsius;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import static edu.wpi.first.units.Units.*;

public class IndexerIOSpark implements IndexerIO {

	private final SparkMax leftMotor = new SparkMax(IndexerConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
	private final SparkMax rightMotor = new SparkMax(IndexerConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);

	private final ArmFeedforward leftFF = new ArmFeedforward(
		IndexerConstants.LEFT_kS,
		IndexerConstants.LEFT_kG,
		IndexerConstants.LEFT_kV,
		IndexerConstants.LEFT_kA
	);

	private final ArmFeedforward rightFF = new ArmFeedforward(
		IndexerConstants.RIGHT_kS,
		IndexerConstants.RIGHT_kG,
		IndexerConstants.RIGHT_kV,
		IndexerConstants.RIGHT_kA
	);

	public IndexerIOSpark() {
		this.configureLeftMotor();	
		this.configureRightMotor();
	}
	
	private void configureLeftMotor() {
		SparkMaxConfig config = new SparkMaxConfig();

		config
			.inverted(false)
			.idleMode(IdleMode.kBrake);
		config.encoder
			.positionConversionFactor(IndexerConstants.PULLEY_CONVERSION_FACTOR)
			.velocityConversionFactor(IndexerConstants.PULLEY_CONVERSION_FACTOR / 60.0);
		config.closedLoop
			.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
				.p(IndexerConstants.LEFT_kP)
				.i(IndexerConstants.LEFT_kI)
				.d(IndexerConstants.LEFT_kD);

		leftMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
	}
	private void configureRightMotor() {
		SparkMaxConfig config = new SparkMaxConfig();

		config
			.inverted(false)
			.idleMode(IdleMode.kBrake);
		config.encoder
			.positionConversionFactor(IndexerConstants.PULLEY_CONVERSION_FACTOR)
			.velocityConversionFactor(IndexerConstants.PULLEY_CONVERSION_FACTOR / 60.0);
		config.closedLoop
			.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
				.p(IndexerConstants.LEFT_kP)
				.i(IndexerConstants.LEFT_kI)
				.d(IndexerConstants.LEFT_kD);

		leftMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
	}

	@Override
	public void updateInputs(IndexerIOInputs inputs) {
		inputs.leftVelocity = RadiansPerSecond.of(leftMotor.getEncoder().getVelocity());
		inputs.leftAppliedVoltage = Volts.of(leftMotor.getAppliedOutput() * leftMotor.getBusVoltage());
		inputs.leftSupplyCurrent = Amps.of(leftMotor.getOutputCurrent());
		inputs.leftTemperature = Celsius.of(leftMotor.getMotorTemperature());

		inputs.rightPosition = Radians.of(leftMotor.getEncoder().getPosition());
		inputs.rightVelocity = RadiansPerSecond.of(leftMotor.getEncoder().getVelocity());
		inputs.rightAppliedVoltage = Volts.of(leftMotor.getAppliedOutput() * leftMotor.getBusVoltage());
		inputs.rightSupplyCurrent = Amps.of(leftMotor.getOutputCurrent());
		inputs.rightTemperature = Celsius.of(leftMotor.getMotorTemperature());
	}

	@Override
	public void stopLeft() {
		leftMotor.setVoltage(0);
	}

	@Override	
	public void stopRight() {
		rightMotor.setVoltage(0);
	}

	@Override
	public void setLeftVoltage(Voltage voltage) {
		leftMotor.setVoltage(voltage);
	}

	@Override
	public void setRightVoltage(Voltage voltage) {
		rightMotor.setVoltage(voltage);
	}

	@Override
	public void setLeftPositionSetpoint(Angle position) {
		double positionRadians = position.in(Radians);
		double feedforward = leftFF.calculate(positionRadians, 0);

		leftMotor.getClosedLoopController().setReference(
			positionRadians,
			ControlType.kPosition,
			ClosedLoopSlot.kSlot0,
			feedforward,
			ArbFFUnits.kVoltage
		);
	}

	@Override
	public void setRightPositionSetpoint(Angle position) {
		double positionRadians = position.in(Radians);
		double feedforward = rightFF.calculate(positionRadians, 0);

		rightMotor.getClosedLoopController().setReference(
			positionRadians,
			ControlType.kPosition,
			ClosedLoopSlot.kSlot0,
			feedforward,
			ArbFFUnits.kVoltage
		);
	}
}