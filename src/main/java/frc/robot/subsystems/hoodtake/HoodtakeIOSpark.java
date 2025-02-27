// CopytopMotor (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hoodtake;

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
import frc.robot.util.SparkUtil;

import static edu.wpi.first.units.Units.*;

public class HoodtakeIOSpark implements HoodtakeIO {

	private final SparkMax wheelMotor = new SparkMax(HoodtakeConstants.WHEEL_MOTOR_ID, MotorType.kBrushless);
	private final SparkMax pivotMotor = new SparkMax(HoodtakeConstants.PIVOT_MOTOR_ID, MotorType.kBrushless);

	private final ArmFeedforward pivotFF = new ArmFeedforward(
			HoodtakeConstants.PIVOT_kS,
			HoodtakeConstants.PIVOT_kG,
			HoodtakeConstants.PIVOT_kV,
			HoodtakeConstants.PIVOT_kA);

	private final SimpleMotorFeedforward wheelFF = new SimpleMotorFeedforward(
			HoodtakeConstants.WHEEL_kS,
			HoodtakeConstants.WHEEL_kV,
			HoodtakeConstants.WHEEL_kA);

	public HoodtakeIOSpark() {
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
				.positionConversionFactor(HoodtakeConstants.WHEEL_CONVERSION_FACTOR)
				.velocityConversionFactor(HoodtakeConstants.WHEEL_CONVERSION_FACTOR / 60.0);
		config.closedLoop
				.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
				.p(HoodtakeConstants.WHEEL_kP)
				.i(HoodtakeConstants.WHEEL_kI)
				.d(HoodtakeConstants.WHEEL_kD);

		wheelMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

	}

	private void configurePivotMotor() {
		SparkMaxConfig config = new SparkMaxConfig();

		config
				.inverted(false)
				.idleMode(IdleMode.kBrake);
		config.encoder
				.positionConversionFactor(HoodtakeConstants.PIVOT_CONVERSION_FACTOR)
				.velocityConversionFactor(HoodtakeConstants.PIVOT_CONVERSION_FACTOR / 60.0);
		config.closedLoop
				.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
				.p(HoodtakeConstants.PIVOT_kP)
				.i(HoodtakeConstants.PIVOT_kI)
				.d(HoodtakeConstants.PIVOT_kD);

		pivotMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		SparkUtil.tryUntilOk(
			pivotMotor,
			5,
			() -> pivotMotor.getEncoder().setPosition(
				HoodtakeConstants.STARTING_ANGLE.in(Radians)
			)
		);
	}

	@Override
	public void updateInputs(HoodtakeIOInputs inputs) {
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
	public void setWheelVelocitySetpoint(AngularVelocity velocity) {
		double velocityRadiansPerSecond = velocity.in(RadiansPerSecond);
		double feedforward = wheelFF.calculate(velocityRadiansPerSecond);

		wheelMotor.getClosedLoopController().setReference(
				velocityRadiansPerSecond,
				ControlType.kVelocity,
				ClosedLoopSlot.kSlot0,
				feedforward,
				ArbFFUnits.kVoltage);
	}

	@Override
	public void setPivotPositionSetpoint(Angle position) {
		double positionRadians = position.in(Radians);
		double feedforward = pivotFF.calculate(positionRadians, 0);

		pivotMotor.getClosedLoopController().setReference(
				positionRadians,
				ControlType.kPosition,
				ClosedLoopSlot.kSlot0,
				feedforward,
				ArbFFUnits.kVoltage);
	}

	@Override
	public void setPivotVoltage(Voltage volts) {
		this.pivotMotor.setVoltage(volts);
	}
}