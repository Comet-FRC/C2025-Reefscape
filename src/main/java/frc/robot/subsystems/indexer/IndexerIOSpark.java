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
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import static edu.wpi.first.units.Units.*;

public class IndexerIOSpark implements IndexerIO {

	private final SparkMax wheelMotor = new SparkMax(IndexerConstants.WHEEL_MOTOR_ID, MotorType.kBrushless);
	private final SparkMax pivotMotor = new SparkMax(IndexerConstants.PIVOT_MOTOR_ID, MotorType.kBrushless);

	private final ArmFeedforward pivotFF = new ArmFeedforward(
		IndexerConstants.PIVOT_kS,
		IndexerConstants.PIVOT_kG,
		IndexerConstants.PIVOT_kV,
		IndexerConstants.PIVOT_kA
	);

	private final SimpleMotorFeedforward wheelFF = new SimpleMotorFeedforward(
		IndexerConstants.WHEEL_kS,
		IndexerConstants.WHEEL_kV,
		IndexerConstants.WHEEL_kA
	);

	public IndexerIOSpark() {
		this.configureWheelMotor();	
		this.configurePivotMotor();
	}

	private void configureWheelMotor() {
		SparkMaxConfig config = new SparkMaxConfig();

		config
			.inverted(false)
			.idleMode(IdleMode.kCoast)
			.smartCurrentLimit(30);
		config.encoder
			.positionConversionFactor(IndexerConstants.WHEEL_CONVERSION_FACTOR)
			.velocityConversionFactor(IndexerConstants.WHEEL_CONVERSION_FACTOR / 60.0);
		config.closedLoop
			.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
				.p(IndexerConstants.WHEEL_kP)
				.i(IndexerConstants.WHEEL_kI)
				.d(IndexerConstants.WHEEL_kD);

		wheelMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

	}

	private void configurePivotMotor() {
		SparkMaxConfig config = new SparkMaxConfig();

		config
			.inverted(false)
			.idleMode(IdleMode.kBrake);
		config.encoder
			.positionConversionFactor(IndexerConstants.PIVOT_CONVERSION_FACTOR)
			.velocityConversionFactor(IndexerConstants.PIVOT_CONVERSION_FACTOR / 60.0);
		config.closedLoop
			.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
				.p(IndexerConstants.PIVOT_kP)
				.i(IndexerConstants.PIVOT_kI)
				.d(IndexerConstants.PIVOT_kD);

		pivotMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
	}

	@Override
	public void updateInputs(IndexerIOInputs inputs) {
		inputs.wheelVelocity = RadiansPerSecond.of(wheelMotor.getEncoder().getVelocity());
		inputs.wheelAppliedVolts = Volts.of(wheelMotor.getAppliedOutput() * wheelMotor.getBusVoltage());
		inputs.wheelSupplyCurrent = Amps.of(wheelMotor.getOutputCurrent());
		inputs.wheelMotorTemperature = Celsius.of(wheelMotor.getMotorTemperature());

		inputs.pivotPosition = Radians.of(wheelMotor.getEncoder().getPosition());
		inputs.wheelVelocity = RadiansPerSecond.of(wheelMotor.getEncoder().getVelocity());
		inputs.wheelAppliedVolts = Volts.of(wheelMotor.getAppliedOutput() * wheelMotor.getBusVoltage());
		inputs.wheelSupplyCurrent = Amps.of(wheelMotor.getOutputCurrent());
		inputs.wheelMotorTemperature = Celsius.of(wheelMotor.getMotorTemperature());
	}

	@Override
	public void stopWheel() {
		wheelMotor.setVoltage(0);
	}

	@Override	
	public void stopPivot() {
		pivotMotor.setVoltage(0);
	}

	@Override
	public void setWheelVoltage(Voltage voltage) {
		wheelMotor.setVoltage(voltage);
	}
	
	@Override
	public void setWheelVelocity(AngularVelocity velocity) {
		double velocityRadiansPerSecond = velocity.in(RadiansPerSecond);
		double feedforward = wheelFF.calculate(velocityRadiansPerSecond);

		wheelMotor.getClosedLoopController().setReference(
			velocityRadiansPerSecond,
			ControlType.kVelocity,
			ClosedLoopSlot.kSlot0,
			feedforward,
			ArbFFUnits.kVoltage
		);
	}

	@Override
	public void setPivotPosition(Angle position) {
		double positionRadians = position.in(Radians);
		double feedforward = pivotFF.calculate(positionRadians, 0);

		pivotMotor.getClosedLoopController().setReference(
			positionRadians,
			ControlType.kPosition,
			ClosedLoopSlot.kSlot0,
			feedforward,
			ArbFFUnits.kVoltage
		);
	}
}