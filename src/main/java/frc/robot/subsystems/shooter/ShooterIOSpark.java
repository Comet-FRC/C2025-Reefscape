// CopytopMotor (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngularVelocity;

import com.revrobotics.spark.SparkMax;

import static edu.wpi.first.units.Units.*;

public class ShooterIOSpark implements ShooterIO {
	private final SparkMax topMotor = new SparkMax(ShooterConstants.TOP_MOTOR_ID, MotorType.kBrushless);
	private final SparkMax bottomMotor = new SparkMax(ShooterConstants.BOTTOM_MOTOR_ID, MotorType.kBrushless);

		private final SimpleMotorFeedforward WheelFF = new SimpleMotorFeedforward(
		ShooterConstants.WheelkS,
		ShooterConstants.WheelkV,
		ShooterConstants.WheelkA
	);

	public ShooterIOSpark() {
		this.configureTopMotor();
		this.configureBottomMotor();
	}

	private void configureTopMotor() {
		SparkMaxConfig config = new SparkMaxConfig();
	 
		config
			.inverted(false)
			.idleMode(IdleMode.kCoast)
			.smartCurrentLimit(30);
		config.encoder
			.positionConversionFactor(ShooterConstants.WHEEL_CONVERSION_FACTOR)
			.velocityConversionFactor(ShooterConstants.WHEEL_CONVERSION_FACTOR / 60.0);
		config.closedLoop
			.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
				.p(ShooterConstants.WheelkP)
				.i(ShooterConstants.WheelkI)
				.d(ShooterConstants.WheelkD);

		topMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
	}
	private void configureBottomMotor() {
		SparkMaxConfig config = new SparkMaxConfig();
	 
		config
			.inverted(false)
			.idleMode(IdleMode.kCoast)
			.smartCurrentLimit(30);
		config.encoder
			.positionConversionFactor(ShooterConstants.WHEEL_CONVERSION_FACTOR)
			.velocityConversionFactor(ShooterConstants.WHEEL_CONVERSION_FACTOR / 60.0);
		config.closedLoop
			.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
				.p(ShooterConstants.WheelkP)
				.i(ShooterConstants.WheelkP)
				.d(ShooterConstants.WheelkP);

		bottomMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
	}
	MutAngularVelocity topWheelDesiredVelocity = RadiansPerSecond.mutable(0);
	MutAngularVelocity bottomWheelDesiredVelocity = RadiansPerSecond.mutable(0);
	@Override
	public void updateInputs(ShooterIOInputs inputs) {
		inputs.topWheelVelocity = RadiansPerSecond.of(topMotor.getEncoder().getVelocity());
		inputs.topWheelAppliedVoltage = Volts.of(topMotor.getAppliedOutput() * topMotor.getBusVoltage());
		inputs.topWheelSupplyCurrent = Amps.of(topMotor.getOutputCurrent());
		inputs.topTemperature = Celsius.of(topMotor.getMotorTemperature());
		inputs.topWheelDesiredVelocity = topWheelDesiredVelocity.copy();
		
		inputs.bottomWheelVelocity = RadiansPerSecond.of(bottomMotor.getEncoder().getVelocity());
		inputs.bottomWheelAppliedVoltage = Volts.of(bottomMotor.getAppliedOutput() * bottomMotor.getBusVoltage());
		inputs.bottomWheelSupplyCurrent = Amps.of(bottomMotor.getOutputCurrent());
		inputs.bottomWheelTemperature= Celsius.of(bottomMotor.getMotorTemperature());
		inputs.bottomWheelDesiredVelocity = this.bottomWheelDesiredVelocity.copy();
	}

	@Override
	public void setWheelVelocitySetpoint(AngularVelocity topVelocity, AngularVelocity bottomVelocity) {
		double topVelocityRadiansPerSecond = topVelocity.in(RadiansPerSecond);
		double topFeedForward = WheelFF.calculate(topVelocityRadiansPerSecond);

		double bottomVelocityRadiansPerSecond = bottomVelocity.in(RadiansPerSecond);
		double bottomFeedForward = WheelFF.calculate(bottomVelocityRadiansPerSecond);

		topMotor.getClosedLoopController().setReference(
			topVelocityRadiansPerSecond,
			ControlType.kVelocity,
			ClosedLoopSlot.kSlot0,
			topFeedForward,
			ArbFFUnits.kVoltage
		);

		bottomMotor.getClosedLoopController().setReference(
			bottomVelocityRadiansPerSecond,
			ControlType.kVelocity,
			ClosedLoopSlot.kSlot0,
			bottomFeedForward,
			ArbFFUnits.kVoltage
		);

		this.topWheelDesiredVelocity.mut_replace(topVelocity);
		this.bottomWheelDesiredVelocity.mut_replace(bottomVelocity);	
	}

	@Override
	public void stop() {
		this.setWheelVelocitySetpoint(RadiansPerSecond.of(0), RadiansPerSecond.of(0));
	}

}
