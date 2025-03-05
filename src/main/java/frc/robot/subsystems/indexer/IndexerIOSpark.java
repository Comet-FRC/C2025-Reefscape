// CopytopMotor (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

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
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.hoodtake.HoodtakeConstants;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.util.SparkUtil;

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
			.idleMode(IdleMode.kCoast);
		config.encoder
			.positionConversionFactor(IndexerConstants.PULLEY_CONVERSION_FACTOR)
			.velocityConversionFactor(IndexerConstants.PULLEY_CONVERSION_FACTOR / 60.0);
		config.closedLoop
			.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
				.p(IndexerConstants.LEFT_kP)
				.i(IndexerConstants.LEFT_kI)
				.d(IndexerConstants.LEFT_kD);

		leftMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
	
		SparkUtil.tryUntilOk(
			leftMotor,
			5,
			() -> leftMotor.getEncoder().setPosition(
				0
			)
		);
	}
	private void configureRightMotor() {
		SparkMaxConfig config = new SparkMaxConfig();

		config
			.inverted(true)
			.idleMode(IdleMode.kCoast);
		config.encoder
			.positionConversionFactor(IndexerConstants.PULLEY_CONVERSION_FACTOR)
			.velocityConversionFactor(IndexerConstants.PULLEY_CONVERSION_FACTOR / 60.0);
		config.closedLoop
			.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
				.p(IndexerConstants.LEFT_kP)
				.i(IndexerConstants.LEFT_kI)
				.d(IndexerConstants.LEFT_kD);

		rightMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
	
		SparkUtil.tryUntilOk(
			rightMotor,
			5,
			() -> rightMotor.getEncoder().setPosition(
				0
			)
		);
	}

	private Voltage leftVoltage = Volts.mutable(0);
	private Voltage rightVoltage = Volts.mutable(0);

	private MutVoltage leftRampingVoltage = Volts.mutable(0);
	private MutVoltage rightRampingVoltage = Volts.mutable(0);

	private boolean leftVoltageMode = false;
	private boolean rightVoltageMode = false;

	@Override
	public void updateInputs(IndexerIOInputs inputs) {
		if (leftVoltageMode) {
			if (leftVoltage.minus(leftRampingVoltage.copy()).gt(Volts.of(1/0.02)))
				leftRampingVoltage.mut_plus(Volts.of(1/0.02));
			else
				leftRampingVoltage.mut_replace(leftVoltage);
			leftMotor.setVoltage(leftRampingVoltage.in(Volts));
		}
		
		if (rightVoltageMode) {
			if (rightVoltage.minus(rightRampingVoltage.copy()).gt(Volts.of(1/0.02)))
				rightRampingVoltage.mut_plus(Volts.of(1/0.02));
			else
				rightRampingVoltage.mut_replace(rightVoltage);
			rightMotor.setVoltage(rightRampingVoltage.in(Volts));
		}

		inputs.leftPosition = Radians.of(leftMotor.getEncoder().getPosition());
		inputs.leftVelocity = RadiansPerSecond.of(leftMotor.getEncoder().getVelocity());
		inputs.leftAppliedVoltage = Volts.of(leftMotor.getAppliedOutput() * leftMotor.getBusVoltage());
		inputs.leftSupplyCurrent = Amps.of(leftMotor.getOutputCurrent());
		inputs.leftTemperature = Celsius.of(leftMotor.getMotorTemperature());

		inputs.rightPosition = Radians.of(rightMotor.getEncoder().getPosition());
		inputs.rightVelocity = RadiansPerSecond.of(rightMotor.getEncoder().getVelocity());
		inputs.rightAppliedVoltage = Volts.of(rightMotor.getAppliedOutput() * rightMotor.getBusVoltage());
		inputs.rightSupplyCurrent = Amps.of(rightMotor.getOutputCurrent());
		inputs.rightTemperature = Celsius.of(rightMotor.getMotorTemperature());
	}

	@Override
	public void stopLeft() {
		this.setLeftVoltage(Volts.of(0));
	}

	@Override	
	public void stopRight() {
		this.setRightVoltage(Volts.of(0));
	}

	@Override
	public void setLeftVoltage(Voltage voltage) {
		this.leftVoltage = voltage;
		this.leftVoltageMode = true;
	}

	@Override
	public void setRightVoltage(Voltage voltage) {
		this.rightVoltage = voltage;
		this.rightVoltageMode = true;
	}

	@Override
	public void setLeftPositionSetpoint(Angle position) {
		this.leftVoltageMode = false;
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
		this.rightVoltageMode = false;
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