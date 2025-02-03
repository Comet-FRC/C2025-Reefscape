// CopytopMotor (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.Measure;

import com.revrobotics.spark.SparkMax;

import static edu.wpi.first.units.Units.*;

public class ShooterIOSpark implements ShooterIO {
	private final SparkMax topMotor = new SparkMax(ShooterConstants.TOP_MOTOR_ID, MotorType.kBrushless);
	private final SparkMax botMotor = new SparkMax(ShooterConstants.BOTTOM_MOTOR_ID, MotorType.kBrushless);

	private ShooterSpeed desiredShooterSpeed = new ShooterSpeed(RPM.of(0),RPM.of(0));

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
				.p(ShooterConstants.kP)
				.i(ShooterConstants.kI)
				.d(ShooterConstants.kD);

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
				.p(ShooterConstants.kP)
				.i(ShooterConstants.kI)
				.d(ShooterConstants.kD);

		botMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
	}
	
	@Override
	public void updateInputs(ShooterIOInputs inputs) {
		inputs.topVelocity = RadiansPerSecond.of(topMotor.getEncoder().getVelocity());
		inputs.topAppliedVoltage = Volts.of(topMotor.getAppliedOutput() * topMotor.getBusVoltage());
		inputs.topSupplyCurrent = Amps.of(topMotor.getOutputCurrent());
		inputs.topTemperature = Celsius.of(topMotor.getMotorTemperature());
		inputs.topDesiredVelocity = desiredShooterSpeed.topMotorSpeed;
		
		inputs.bottomVelocity = RadiansPerSecond.of(botMotor.getEncoder().getVelocity());
		inputs.bottomAppliedVoltage = Volts.of(botMotor.getAppliedOutput() * botMotor.getBusVoltage());
		inputs.bottomSupplyCurrent = Amps.of(botMotor.getOutputCurrent());
		inputs.bottomTemperature= Celsius.of(botMotor.getMotorTemperature());
		inputs.bottomDesiredVelocity = desiredShooterSpeed.botMotorSpeed;
	}

	@Override
	public void setAngularVelocity(ShooterSpeed shooterSpeed) {
		topMotor.getClosedLoopController().setReference(shooterSpeed.topMotorSpeed.in(RadiansPerSecond), ControlType.kVelocity);
		botMotor.getClosedLoopController().setReference(shooterSpeed.botMotorSpeed.in(RadiansPerSecond), ControlType.kVelocity);
	}

	@Override
	public void stop() {
		this.setAngularVelocity(new ShooterSpeed(RadiansPerSecond.of(0), RadiansPerSecond.of(0)));
	}


}
