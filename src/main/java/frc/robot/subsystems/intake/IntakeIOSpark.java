// CopytopMotor (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.indexer.IndexerConstants;
import frc.robot.util.SparkUtil;

import static edu.wpi.first.units.Units.*;

public class IntakeIOSpark implements IntakeIO {

	private final SparkMax wheelMotor = new SparkMax(IntakeConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);
	private final SparkMax pivotMotor = new SparkMax(IntakeConstants.PIVOT_MOTOR_ID, MotorType.kBrushless);

	private final ArmFeedforward pivotFF = new ArmFeedforward(
		IntakeConstants.PIVOT_kS,
		IntakeConstants.PIVOT_kG,
		IntakeConstants.PIVOT_kV,
		IntakeConstants.PIVOT_kA
	);

	private final SimpleMotorFeedforward wheelFF = new SimpleMotorFeedforward(
		IntakeConstants.WHEEL_kS,
		IntakeConstants.WHEEL_kV,
		IntakeConstants.WHEEL_kA
	);

	public IntakeIOSpark() {
		this.configureWheelMotor();	
		this.configurePivotMotor();
	}

	private void configureWheelMotor() {
		SparkMaxConfig config = new SparkMaxConfig();

		config
			.inverted(false)
			.idleMode(IdleMode.kCoast)
			.smartCurrentLimit(20);
		config.encoder
			.positionConversionFactor(IntakeConstants.WHEEL_CONVERSION_FACTOR)
			.velocityConversionFactor(IntakeConstants.WHEEL_CONVERSION_FACTOR / 60.0);
		config.closedLoop
			.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
				.p(IntakeConstants.WHEEL_kP)
				.i(IntakeConstants.WHEEL_kI)
				.d(IntakeConstants.WHEEL_kD);

		wheelMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

	}
	private void configurePivotMotor() {
		SparkMaxConfig config = new SparkMaxConfig();

		config
			.inverted(false)
			.idleMode(IdleMode.kBrake)
			.smartCurrentLimit(40); // TODO: Check if this is enough current
		config.encoder
			.positionConversionFactor(IntakeConstants.PIVOT_CONVERSION_FACTOR)
			.velocityConversionFactor(IntakeConstants.PIVOT_CONVERSION_FACTOR / 60.0);
		config.closedLoop
			.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
				.p(IntakeConstants.PIVOT_kP)
				.i(IntakeConstants.PIVOT_kI)
				.d(IntakeConstants.PIVOT_kD);
		config.signals
			.primaryEncoderPositionAlwaysOn(true)
			.primaryEncoderPositionPeriodMs(20)
			.primaryEncoderVelocityAlwaysOn(true)
			.primaryEncoderVelocityPeriodMs(10)
			.appliedOutputPeriodMs(20)
			.busVoltagePeriodMs(20)
			.outputCurrentPeriodMs(20);

		pivotMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		SparkUtil.tryUntilOk(
			pivotMotor,
			5,
			() -> pivotMotor.getEncoder().setPosition(
				IntakeConstants.STARTING_ANGLE.in(Radians)
			)
		);
	}


	
	MutAngularVelocity wheelDesiredVelocity = RadiansPerSecond.mutable(0);
	MutVoltage wheelDesiredVoltage = Volts.mutable(0);
	boolean wheelVoltageMode = false;

	MutAngle pivotDesiredPosition = IntakeConstants.STARTING_ANGLE.mutableCopy();
	MutVoltage pivotDesiredVoltage = Volts.mutable(0);
	boolean pivotVoltageMode = false;

	@Override
	public void updateInputs(IntakeIOInputs inputs) {
		if (pivotVoltageMode) {
			this.pivotMotor.setVoltage(pivotDesiredVoltage.in(Volts));
		}

		if (wheelVoltageMode) {
			this.wheelMotor.setVoltage(wheelDesiredVoltage.in(Volts));
		}

		inputs.wheelPosition = Radians.of(wheelMotor.getEncoder().getPosition());
		inputs.wheelVelocity = RadiansPerSecond.of(wheelMotor.getEncoder().getVelocity());
		inputs.wheelDesiredVelocity = this.wheelDesiredVelocity.copy();
		inputs.wheelAppliedVoltage = Volts.of(wheelMotor.getAppliedOutput() * wheelMotor.getBusVoltage());
		inputs.wheelSupplyCurrent = Amps.of(wheelMotor.getOutputCurrent());
		inputs.wheelMotorTemperature = Celsius.of(wheelMotor.getMotorTemperature());

		inputs.pivotPosition = Radians.of(pivotMotor.getEncoder().getPosition());
		inputs.pivotDesiredPosition = this.pivotDesiredPosition.copy();
		inputs.pivotVelocity = RadiansPerSecond.of(pivotMotor.getEncoder().getVelocity());
		inputs.pivotAppliedVoltage = Volts.of(pivotMotor.getAppliedOutput() * pivotMotor.getBusVoltage());
		inputs.pivotSupplyCurrent = Amps.of(pivotMotor.getOutputCurrent());
		inputs.pivotTemperature = Celsius.of(pivotMotor.getMotorTemperature());
	}

	@Override
	public void stopWheel() {
		this.setWheelVoltage(Volts.of(0));
	}

	@Override	
	public void stopPivot() {
		this.setPivotVoltage(Volts.of(0));
	}

	@Override
	public void setWheelVoltage(Voltage voltage) {
		this.wheelVoltageMode = true;
		this.wheelDesiredVoltage.mut_replace(voltage);
	}
	
	@Override
	public void setWheelVelocitySetpoint(AngularVelocity velocity) {
		this.wheelVoltageMode = false;

		double velocityRadiansPerSecond = velocity.in(RadiansPerSecond);
		double feedforward = wheelFF.calculate(velocityRadiansPerSecond);

		wheelMotor.getClosedLoopController().setReference(
			velocityRadiansPerSecond,
			ControlType.kVelocity,
			ClosedLoopSlot.kSlot0,
			feedforward,
			ArbFFUnits.kVoltage
		);

		this.wheelDesiredVelocity.mut_replace(velocity);
	}

	@Override
	public void setPivotPositionSetpoint(Angle position) {
		this.pivotVoltageMode = false;

		double positionRadians = position.in(Radians);
		double feedforward = pivotFF.calculate(positionRadians, 0);

		pivotMotor.getClosedLoopController().setReference(
			positionRadians,
			ControlType.kPosition,
			ClosedLoopSlot.kSlot0,
			feedforward,
			ArbFFUnits.kVoltage
		);

		this.pivotDesiredPosition.mut_replace(position);
	}

	@Override
	public void setPivotVoltage(Voltage volts) {
		this.pivotVoltageMode = true;
		this.pivotDesiredVoltage.mut_replace(volts);
	}
}