// CopytopMotor (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hoodtake;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
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

	private final ProfiledPIDController pivotPID = new ProfiledPIDController(
		HoodtakeConstants.PIVOT_kP,
		HoodtakeConstants.PIVOT_kI,
		HoodtakeConstants.PIVOT_kD,
		new TrapezoidProfile.Constraints(5, 10)
	);

	private final PIDController wheelPID = new PIDController(
		HoodtakeConstants.WHEEL_kP,
		HoodtakeConstants.WHEEL_kI,
		HoodtakeConstants.WHEEL_kD
	);

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
		// config.closedLoop
		// 		.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
		// 		.p(HoodtakeConstants.WHEEL_kP)
		// 		.i(HoodtakeConstants.WHEEL_kI)
		// 		.d(HoodtakeConstants.WHEEL_kD);

		wheelMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

	}

	private void configurePivotMotor() {
		SparkMaxConfig config = new SparkMaxConfig();

		config
				.inverted(true)
				.idleMode(IdleMode.kBrake);
		config.encoder
				.positionConversionFactor(HoodtakeConstants.PIVOT_CONVERSION_FACTOR)
				.velocityConversionFactor(HoodtakeConstants.PIVOT_CONVERSION_FACTOR / 60.0);
		// config.closedLoop
		// 		.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
		// 		.p(HoodtakeConstants.PIVOT_kP)
		// 		.i(HoodtakeConstants.PIVOT_kI)
		// 		.d(HoodtakeConstants.PIVOT_kD)
		// 		.outputRange(Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
		config.signals
                .primaryEncoderPositionAlwaysOn(true)
                .primaryEncoderPositionPeriodMs(20)
                .primaryEncoderVelocityAlwaysOn(true)
                .primaryEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);

		pivotMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		SparkUtil.tryUntilOk(
			pivotMotor,
			5,
			() -> pivotMotor.getEncoder().setPosition(
				HoodtakeConstants.STARTING_ANGLE.in(Radians)
			)
		);
	}

	private final MutVoltage wheelDesiredVoltage = Volts.mutable(0);
	private boolean wheelVoltageMode = false;

	private final MutVoltage pivotDesiredVoltage = Volts.mutable(0);
	private boolean pivotVoltageMode = false;

	@Override
	public void updateInputs(HoodtakeIOInputs inputs) {
		if (pivotVoltageMode) {
			this.pivotMotor.setVoltage(pivotDesiredVoltage.copy());
		} else {
			this.pivotMotor.setVoltage(
				Volts.of(
					pivotPID.calculate(pivotMotor.getEncoder().getPosition())
					+
					pivotFF.calculate(pivotPID.getSetpoint().position, pivotPID.getSetpoint().velocity)
				)
			);
		}

		if (wheelVoltageMode) {
			this.wheelMotor.setVoltage(wheelDesiredVoltage.copy());
		} else {
			this.wheelMotor.setVoltage(
				Volts.of(
					wheelPID.calculate(wheelMotor.getEncoder().getVelocity())
					+
					wheelFF.calculate(wheelPID.getSetpoint(), 0)
				)
			);
		}

		inputs.wheelPosition = Radians.of(wheelMotor.getEncoder().getPosition());
		inputs.wheelVelocity = RadiansPerSecond.of(wheelMotor.getEncoder().getVelocity());
		inputs.wheelDesiredVelocity = RadiansPerSecond.of(wheelPID.getSetpoint());
		inputs.wheelAppliedVolts = Volts.of(wheelMotor.getAppliedOutput() * wheelMotor.getBusVoltage());
		inputs.wheelSupplyCurrent = Amps.of(wheelMotor.getOutputCurrent());
		inputs.wheelMotorTemperature = Celsius.of(wheelMotor.getMotorTemperature());

		inputs.pivotPosition = Radians.of(pivotMotor.getEncoder().getPosition());
		inputs.pivotDesiredPosition = Radians.of(pivotPID.getSetpoint().position);
		inputs.pivotVelocity = RadiansPerSecond.of(pivotMotor.getEncoder().getVelocity());
		inputs.pivotAppliedVolts = Volts.of(pivotMotor.getAppliedOutput() * pivotMotor.getBusVoltage());
		inputs.pivotSupplyCurrent = Amps.of(pivotMotor.getOutputCurrent());
		inputs.pivotTemperature = Celsius.of(pivotMotor.getMotorTemperature());
	}

	@Override
	public void stopWheel() {
		wheelMotor.setVoltage(0);
	}

	@Override
	public void stopPivot() {
		this.setPivotVoltage(Volts.of(0));
	}

	@Override
	public void setWheelVoltage(Voltage voltage) {
		this.wheelVoltageMode = true;
		wheelMotor.setVoltage(voltage);
	}

	@Override
	public void setWheelVelocitySetpoint(AngularVelocity velocity) {
		this.wheelVoltageMode = false;
		this.wheelPID.setSetpoint(velocity.in(RadiansPerSecond));
	}

	@Override
	public void setPivotPositionSetpoint(Angle position) {
		this.pivotVoltageMode = false;
		pivotPID.setGoal(position.in(Radians));
	}

	@Override
	public void setPivotVoltage(Voltage volts) {
		this.pivotVoltageMode = true;
		this.pivotDesiredVoltage.mut_replace(volts);
	}
}