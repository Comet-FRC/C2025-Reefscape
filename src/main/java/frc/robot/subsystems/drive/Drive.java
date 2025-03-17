// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.drive.SwerveConstants.DRIVE_BASE_RADIUS;
import static frc.robot.subsystems.drive.SwerveConstants.MAX_SPEED;
import static frc.robot.subsystems.drive.SwerveConstants.MODULE_TRANSLATIONS;
import static frc.robot.subsystems.drive.SwerveConstants.PATHPLANNER_CONFIG;

import java.util.Set;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.gyro.GyroIO;
import frc.robot.subsystems.drive.gyro.GyroIOInputsAutoLogged;
import frc.robot.subsystems.drive.module.Module;
import frc.robot.subsystems.drive.module.ModuleIO;
import frc.robot.subsystems.drive.module.SparkOdometryThread;
import frc.robot.util.CometMathUtil;
import frc.robot.util.LocalADStarAK;

public class Drive extends SubsystemBase {
	private static Drive instance;

	public static Drive getInstance() {
		if (instance == null) {
			throw new java.lang.RuntimeException("Drive subsystem not instantiated");
		}
		return instance;
	}

	public static final Lock odometryLock = new ReentrantLock();

	private final GyroIO gyroIO;
	private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
	private final Module[] modules = new Module[4]; // FL, FR, BL, BR
	private final SysIdRoutine sysId;

	private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(MODULE_TRANSLATIONS);
	private Rotation2d rawGyroRotation = new Rotation2d();
	private SwerveModulePosition[] lastModulePositions = // For delta tracking
			new SwerveModulePosition[] {
					new SwerveModulePosition(),
					new SwerveModulePosition(),
					new SwerveModulePosition(),
					new SwerveModulePosition()
			};

	private Field2d field = new Field2d();
		
	private SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
			kinematics, rawGyroRotation, lastModulePositions, new Pose2d(3, 3, new Rotation2d()));

	private ProfiledPIDController headingPID = new ProfiledPIDController(
			DriveConstants.HEADING_kP,
			DriveConstants.HEADING_kI,
			DriveConstants.HEADING_kD,
			new TrapezoidProfile.Constraints(Math.PI, Math.PI)
	);

	private ProfiledPIDController headingCorrectionPID = new ProfiledPIDController(
			DriveConstants.HEADING_kP,
			DriveConstants.HEADING_kI,
			DriveConstants.HEADING_kD,
			new TrapezoidProfile.Constraints(Math.PI, Math.PI)
	);

	private PIDController xPID = new PIDController(
			DriveConstants.TRANSLATION_kP,
			DriveConstants.TRANSLATION_kI,
			DriveConstants.TRANSLATION_kD);

	private PIDController yPID = new PIDController(
			DriveConstants.TRANSLATION_kP,
			DriveConstants.TRANSLATION_kI,
			DriveConstants.TRANSLATION_kD);

	/** true if translation control is overridden */
	boolean isXPIDEnabled = false;
	boolean isYPIDEnabled = false;
	/** true if heading control is overridden */
	boolean isHeadingPIDEnabled = false;

	ChassisSpeeds targetChassisSpeeds = new ChassisSpeeds();

	TargetAlgae targetAlgae = null;
	Angle lastHeadingRadians = Angle.ofBaseUnits(0, Radians);

	public Drive(
			GyroIO gyroIO,
			ModuleIO flModuleIO,
			ModuleIO frModuleIO,
			ModuleIO blModuleIO,
			ModuleIO brModuleIO) {

		instance = this;

		this.gyroIO = gyroIO;
		this.modules[0] = new Module(flModuleIO, 0);
		this.modules[1] = new Module(frModuleIO, 1);
		this.modules[2] = new Module(blModuleIO, 2);
		this.modules[3] = new Module(brModuleIO, 3);

		SparkOdometryThread.getInstance().start();

		// Configure AutoBuilder for PathPlanner
		AutoBuilder.configure(
				this::getPose,
				this::setPose,
				this::getChassisSpeeds,
				this::runVelocity,
				new PPHolonomicDriveController(
						new PIDConstants(
								DriveConstants.TRANSLATION_kP,
								DriveConstants.TRANSLATION_kI,
								DriveConstants.TRANSLATION_kD),
						new PIDConstants(
								DriveConstants.HEADING_kP,
								DriveConstants.HEADING_kI,
								DriveConstants.HEADING_kD)),
				PATHPLANNER_CONFIG,
				() -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
				this);
		Pathfinding.setPathfinder(new LocalADStarAK());
		PathPlannerLogging.setLogActivePathCallback(
				(activePath) -> {
					Logger.recordOutput(
							"Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
				});
		PathPlannerLogging.setLogTargetPoseCallback(
				(targetPose) -> {
					Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
				});

		// Configure SysId
		sysId = new SysIdRoutine(
				new SysIdRoutine.Config(
						null,
						null,
						null,
						(state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
				new SysIdRoutine.Mechanism((voltage) -> runCharacterizationLinear(voltage), null, this));
	
		this.setupLoopControllers();		
		
		SmartDashboard.putData(field);
	}

	public void setupLoopControllers() {
		this.headingPID.enableContinuousInput(-Math.PI, Math.PI);
		this.headingCorrectionPID.enableContinuousInput(-Math.PI, Math.PI);
		this.xPID.setTolerance(0.1);
		this.yPID.setTolerance(0.1);
		this.headingPID.setTolerance(Units.degreesToRadians(4));
		this.headingPID.reset(this.getPose().getRotation().getRadians());
		this.headingCorrectionPID.reset(this.getPose().getRotation().getRadians());
	}

	@Override
	public void periodic() {
		
		

		if (isXPIDEnabled || isYPIDEnabled) {
			
			targetChassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(targetChassisSpeeds, this.getRotation());

			if (isXPIDEnabled)
				targetChassisSpeeds.vxMetersPerSecond = this.xPID.calculate(getPose().getX());
			
			if (isYPIDEnabled)
				targetChassisSpeeds.vyMetersPerSecond = this.yPID.calculate(getPose().getY());

			targetChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(targetChassisSpeeds, this.getRotation());
		
			// Logger.recordOutput("Drive/xPID Error", this.xPID.getError());
			// Logger.recordOutput("Drive/yPID Error", this.yPID.getError());
		
		}

		if (isHeadingPIDEnabled) {
			targetChassisSpeeds.omegaRadiansPerSecond = this.headingPID.calculate(this.getRotation().getRadians());
			// Logger.recordOutput("Drive/headingPID Error", this.headingPID.getPositionError());
		}

		// HEADING CORRECION
		if (DriveConstants.IS_HEADING_CORRECTION_ENABLED) {
			if (Math.abs(targetChassisSpeeds.omegaRadiansPerSecond) < DriveConstants.HEADING_CORRECTION_DEADBAND
					&& (Math.abs(targetChassisSpeeds.vxMetersPerSecond) > DriveConstants.HEADING_CORRECTION_DEADBAND
							|| Math.abs(
									targetChassisSpeeds.vyMetersPerSecond) > DriveConstants.HEADING_CORRECTION_DEADBAND)) {
				targetChassisSpeeds.omegaRadiansPerSecond = headingCorrectionPID
						.calculate(this.getPose().getRotation().getRadians(), lastHeadingRadians.in(Radians));
			} else {
				lastHeadingRadians = Radians.of(this.getPose().getRotation().getRadians());
				headingCorrectionPID.reset(lastHeadingRadians.in(Radians));
			}
		}

		// ANGULAR VELOCITY CORRECTION
		if (DriveConstants.IS_ANGULAR_VELOCITY_CORRECTION_ENABLED) {
			Rotation2d angularVelocity = new Rotation2d(gyroInputs.yawVelocity.in(RadiansPerSecond))
					.times(DriveConstants.angularVelocityCoefficient);
			if (angularVelocity.getRadians() != 0.0) {
				ChassisSpeeds fieldRelativeVelocity = ChassisSpeeds.fromRobotRelativeSpeeds(targetChassisSpeeds,
						this.getPose().getRotation());
				targetChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeVelocity,
						this.getPose().getRotation().plus(angularVelocity));
			}
		}
		// Calculate module setpoints
		ChassisSpeeds speeds = ChassisSpeeds.discretize(targetChassisSpeeds, 0.02);

		SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(speeds);
		SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, SwerveConstants.MAX_SPEED);

		// Log unoptimized setpoints
		Logger.recordOutput("Swerve/SwerveStates/Setpoints", setpointStates);
		Logger.recordOutput("Swerve/SwerveChassisSpeeds/Setpoints", speeds);

		// Send setpoints to modules
		for (int i = 0; i < 4; i++) {
			modules[i].runSetpoint(setpointStates[i]);
		}

		// Log optimized setpoints (runSetpoint mutates each state)
		Logger.recordOutput("Swerve/SwerveStates/SetpointsOptimized", setpointStates);

		odometryLock.lock(); // Prevents odometry updates while reading data
		gyroIO.updateInputs(gyroInputs);
		Logger.processInputs("Drive/Gyro", gyroInputs);
		for (var module : modules) {
			module.periodic();
		}
		odometryLock.unlock();

		// Stop moving when disabled
		if (DriverStation.isDisabled()) {
			for (var module : modules) {
				module.stop();
			}
		}

		// Log empty setpoint states when disabled
		if (DriverStation.isDisabled()) {
			Logger.recordOutput("Swerve/SwerveStates/Setpoints", new SwerveModuleState[] {});
			Logger.recordOutput("Swerve/SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
		}

		// Update odometry
		double[] sampleTimestamps = modules[0].getOdometryTimestamps(); // All signals are sampled together
		int sampleCount = sampleTimestamps.length;
		for (int i = 0; i < sampleCount; i++) {
			// Read wheel positions and deltas from each module
			SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
			SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
			for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
				modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
				moduleDeltas[moduleIndex] = new SwerveModulePosition(
						modulePositions[moduleIndex].distanceMeters
								- lastModulePositions[moduleIndex].distanceMeters,
						modulePositions[moduleIndex].angle);
				lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
			}

			// Update gyro angle
			if (gyroInputs.connected) {
				// Use the real gyro angle
				rawGyroRotation = gyroInputs.odometryYawPositions[i];
			} else {
				// Use the angle delta from the kinematics and module deltas
				Twist2d twist = kinematics.toTwist2d(moduleDeltas);
				rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
			}

			// Apply update
			poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
		}

		// Update gyro alert
		DriveConstants.ALERT_DISCONNECTED_GYRO.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);

		this.updateTargetAlgae();
		this.field.setRobotPose(getPose());
	}

	/**
	 * Runs the drive at the desired velocity.
	 *
	 * @param speeds Speeds in meters/sec
	 */
	public void runVelocity(ChassisSpeeds speeds) {
		targetChassisSpeeds = speeds;
	}

	/** Runs the drive in a straight line with the specified drive output. */
	public void runDriveVoltage(Voltage output) {
		for (int i = 0; i < 4; i++) {
			modules[i].setDriveVoltage(output);;
		}
	}

	public void setDriveAngleSetpoints(Rotation2d angle) {
		for (int i = 0; i < 4; i++) {
			modules[i].setTurnPosition(angle);
		}
	};
	
	
	public void setDriveAngleSetpointToRotationPattern() {
		for (int i = 0; i < 4; i++) {
			modules[i].setTurnPosition(MODULE_TRANSLATIONS[i].getAngle().plus(Rotation2d.fromDegrees(90)));
		}
	};

	/** Runs the drive in a straight line with the specified drive output. */
	public void runCharacterizationLinear(Voltage output) {
		for (int i = 0; i < 4; i++) {
			modules[i].runCharacterizationLinear(output);
		}
	}

	/** Stops the drive. */
	public void stop() {
		runVelocity(new ChassisSpeeds());
	}

	/**
	 * Stops the drive and turns the modules to an X arrangement to resist movement.
	 * The modules will
	 * return to their normal orientations the next time a nonzero velocity is
	 * requested.
	 */
	public void stopWithX() {
		Rotation2d[] headings = new Rotation2d[4];
		for (int i = 0; i < 4; i++) {
			headings[i] = MODULE_TRANSLATIONS[i].getAngle();
		}
		kinematics.resetHeadings(headings);
		stop();
	}

	/** Returns a command to run a quasistatic test in the specified direction. */
	public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
		return run(() -> runCharacterizationLinear(Volts.of(0.0)))
				.withTimeout(1.0)
				.andThen(sysId.quasistatic(direction));
	}

	/** Returns a command to run a dynamic test in the specified direction. */
	public Command sysIdDynamic(SysIdRoutine.Direction direction) {
		return run(() -> runCharacterizationLinear(Volts.of(0.0)))
				.withTimeout(1.0)
				.andThen(sysId.dynamic(direction));
	}

	/**
	 * Returns the module states (turn angles and drive velocities) for all of the
	 * modules.
	 */
	@AutoLogOutput(key = "Swerve/SwerveStates/Measured")
	private SwerveModuleState[] getModuleStates() {
		SwerveModuleState[] states = new SwerveModuleState[4];
		for (int i = 0; i < 4; i++) {
			states[i] = modules[i].getState();
		}
		return states;
	}

	/**
	 * Returns the module positions (turn angles and drive positions) for all of the
	 * modules.
	 */
	private SwerveModulePosition[] getModulePositions() {
		SwerveModulePosition[] states = new SwerveModulePosition[4];
		for (int i = 0; i < 4; i++) {
			states[i] = modules[i].getSwervePosition();
		}
		return states;
	}

	/** Returns the measured chassis speeds of the robot. */
	@AutoLogOutput(key = "Swerve/SwerveChassisSpeeds/Measured")
	private ChassisSpeeds getChassisSpeeds() {
		return kinematics.toChassisSpeeds(getModuleStates());
	}

	/** Returns the position of each module in radians. */
	public double[] getWheelRadiusCharacterizationPositions() {
		double[] values = new double[4];
		for (int i = 0; i < 4; i++) {
			values[i] = modules[i].getWheelRadiusCharacterizationPosition().in(Radians);
		}
		return values;
	}

	/** Returns the average velocity of the modules in rad/sec. */
	public AngularVelocity getFFCharacterizationVelocity() {
		double output = 0.0;
		for (int i = 0; i < 4; i++) {
			output += modules[i].getFFCharacterizationVelocity().baseUnitMagnitude() / 4.0;
		}
		return RadiansPerSecond.of(output);
	}

	/** Returns the current odometry pose. */
	@AutoLogOutput(key = "Odometry/Robot")
	public Pose2d getPose() {
		return poseEstimator.getEstimatedPosition();
	}

	@AutoLogOutput(key = "Odometry/Gyro/Yaw")
	public Angle getRawGyroAngle() {
		return gyroInputs.yawPosition.getMeasure();
	}

	@AutoLogOutput(key = "Robot/MeasuredSpeed")
	public LinearVelocity getSpeed() {
		return MetersPerSecond.of(
				Math.hypot(
						this.getChassisSpeeds().vxMetersPerSecond, this.getChassisSpeeds().vyMetersPerSecond));
	}

	/** Returns the current odometry rotation. */
	public Rotation2d getRotation() {
		return getPose().getRotation();
	}

	public ChassisSpeeds getFieldOrientedChassisSpeeds() {
		return ChassisSpeeds.fromRobotRelativeSpeeds(getChassisSpeeds(), getRotation());
	}

	/** Resets the current odometry pose. */
	public void resetHeadingWithAlliance() {
		boolean isRedAlliance = DriverStation.getAlliance().isPresent()
				&& DriverStation.getAlliance().get() == Alliance.Red;

		Rotation2d newHeading = isRedAlliance ? new Rotation2d(Degrees.of(180)) : new Rotation2d();
		poseEstimator.resetPosition(
				rawGyroRotation,
				getModulePositions(),
				new Pose2d(this.getPose().getTranslation(), newHeading));
	}

	public void setPose(Pose2d pose) {
		if (pose == null) {
			return;
		}
		poseEstimator.resetPosition(
				rawGyroRotation.plus(pose.getRotation()),
				getModulePositions(),
				pose
		);
	}

	/** Adds a new timestamped vision measurement. */
	public void addVisionMeasurement(
			Pose2d visionRobotPoseMeters,
			double timestampSeconds,
			Matrix<N3, N1> visionMeasurementStdDevs) {
		poseEstimator.addVisionMeasurement(
				visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
	}

	/** Returns the maximum linear speed. */
	public LinearVelocity getMaximumSpeed() {
		return MAX_SPEED;
	}

	/** Returns the maximum angular speed. */
	public AngularVelocity getMaximumAngularSpeed() {
		return RadiansPerSecond.of(MAX_SPEED.in(MetersPerSecond) / DRIVE_BASE_RADIUS.in(Meters));
	}

	public Voltage getAverageModuleDriveAppliedVoltage() {
		double voltage = 0.0;
		for (int i = 0; i < 4; i++) {
			voltage += modules[i].getAppliedVoltage().in(Volts);
		}
		return Volts.of(voltage / 4.0);
	}

	public Angle getAverageDriveAngularPosition() {
		double position = 0.0;
		for (int i = 0; i < 4; i++) {
			position += modules[i].getWheelRadiusCharacterizationPosition().in(Radians);
		}
		return Radians.of(position / 4.0);
	}

	public AngularVelocity getAverageDriveAngularVelocity() {
		double velocity = 0.0;
		for (int i=0; i< 4; ++i) {
			velocity += modules[i].getFFCharacterizationVelocity().in(RadiansPerSecond);
		}
		return RadiansPerSecond.of(velocity/4.0);
	}

	public Distance getDistanceFrom(Translation2d other) {
		return Meters.of(this.getPose().getTranslation().getDistance(other));
	}



	public Distance getDistanceFrom(Pose2d other) {
		return this.getDistanceFrom(other.getTranslation());
	}

	


	// public void initializePoseEstimator(
	// 		SwerveDriveKinematics kinematics,
	// 		Rotation2d gyroAngle,
	// 		SwerveModulePosition[] modulePositions,
	// 		Pose2d initialPoseMeters) {
	// 	poseEstimator = new SwerveDrivePoseEstimator(kinematics, gyroAngle, modulePositions, initialPoseMeters);
	// 	SmartDashboard.putData("field", field);
	// 	logOdometry();
	// }

	public void addDriveMeasurement(Rotation2d rotation, SwerveModulePosition[] modulePositions) {
		poseEstimator.update(rotation, modulePositions);
	}

	public void setPose(Rotation2d rotation, SwerveModulePosition[] modulePositions, Pose2d fieldToVehicle) {
		poseEstimator.resetPosition(rotation, modulePositions, fieldToVehicle);
	}

	// public void logOdometry() {
	// 	Pose2d pose = getPose();
	// 	Logger.recordOutput("Odometry/Robot", pose);
	// 	field.setRobotPose(pose);
	// }

	public boolean isOnOpposingSide() {
		return this.getPose().getX() > FieldConstants.fieldLength / 2.0;
	}

	/**
	 * Field relative drive command using two joysticks (controlling linear and
	 * angular velocities).
	 */
	public Command joystickDrive(
			DoubleSupplier xSupplier,
			DoubleSupplier ySupplier,
			DoubleSupplier omegaSupplier) {
		return Commands.run(
				() -> {
					double x = xSupplier.getAsDouble();
					double y = ySupplier.getAsDouble();

					x = MathUtil.clamp(x, -1.0, 1.0);
					y= MathUtil.clamp(y, -1.0, 1.0);


					// linear
					double linearSpeedScalar;
					linearSpeedScalar = Math.hypot(x, y);

					linearSpeedScalar = MathUtil.clamp(linearSpeedScalar, -1.0, 1.0);
					linearSpeedScalar = MathUtil.applyDeadband(linearSpeedScalar, DriveConstants.JOYSTICK_DEADBAND_LINEAR);


					linearSpeedScalar = CometMathUtil.minMaxScale(
						linearSpeedScalar,
						DriveConstants.JOYSTICK_DEADBAND_LINEAR,
						1
					);

					Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));
					Translation2d linearTranslation = new Translation2d(linearSpeedScalar, linearDirection);

					// angular
					double omegaSpeedScalar;
					omegaSpeedScalar = omegaSupplier.getAsDouble();
					omegaSpeedScalar = MathUtil.clamp(omegaSpeedScalar, -1.0, 1.0);
					omegaSpeedScalar = MathUtil.applyDeadband(omegaSpeedScalar, DriveConstants.JOYSTICK_DEADBAND_ANGULAR);
					omegaSpeedScalar = CometMathUtil.minMaxScale(
						omegaSpeedScalar,
						DriveConstants.JOYSTICK_DEADBAND_ANGULAR,
						1
					);


					
					omegaSpeedScalar = Math.signum(omegaSpeedScalar) * Math.pow(omegaSpeedScalar, 2);
					// omegaSpeedScalar = Math.pow(omegaSpeedScalar, 3);
					// omegaSpeedScalar *= -1;

					ChassisSpeeds speeds = new ChassisSpeeds(
							this.getMaximumSpeed().times(linearTranslation.getX()),
							this.getMaximumSpeed().times(linearTranslation.getY()),
							this.getMaximumAngularSpeed().times(omegaSpeedScalar));


					boolean isFlipped = DriverStation.getAlliance().isPresent()
							&& DriverStation.getAlliance().get() == Alliance.Red;

					Rotation2d robotAngle = isFlipped
							? this.getRotation().plus(new Rotation2d(Degrees.of(180)))
							: this.getRotation();

					speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, robotAngle);

					this.runVelocity(speeds);
				},
				this);
	}

	@AutoLogOutput(key = "Swerve/HeadingPIDSetpoint")
	public Rotation2d getHeadingPIDSetpoint() {
		return Rotation2d.fromRadians(this.headingPID.getSetpoint().position);
	}

	public void updateTargetAlgae() {
		boolean isOpposingReef = this.isOnOpposingSide();
		Pose2d[] algaeLocations = isOpposingReef 
			? FieldConstants.Reef.reefAlgaeTargetPosesOpposingSide 
			: FieldConstants.Reef.reefAlgaeTargetPoses;

		Pose2d closestPose = algaeLocations[0];
		int closestAlgaeIndex = 0;
		Distance closestDistance = this.getDistanceFrom(closestPose);

		for (int i = 1; i < 6; ++i) {
			Pose2d algaePose = algaeLocations[i];

			Distance algaeDistance = this.getDistanceFrom(algaePose);

			if (algaeDistance.lt(closestDistance)) {
				closestPose = algaePose;
				closestAlgaeIndex = i;
				closestDistance = algaeDistance;
			}
		}

		this.targetAlgae = new TargetAlgae(closestPose, closestAlgaeIndex, isOpposingReef);
	}

	@AutoLogOutput(key = "Automation/TargetAlgae")
	public TargetAlgae getTargetAlgae() {
		return targetAlgae;
	}

	public Command pathfindToPose(Supplier<Pose2d> pose, double goalEndVelocity) {
		return Commands.defer(
			() -> AutoBuilder.pathfindToPose(
					pose.get(),
					new PathConstraints(
							this.getMaximumSpeed(),
							MetersPerSecondPerSecond.of(4),
							this.getMaximumAngularSpeed(),
							DegreesPerSecondPerSecond.of(720)),
							goalEndVelocity),
			Set.of(this))
			.andThen(() -> this.stop());
	}

	public Command driveToClosestAlgaePID(Supplier<Distance> distance) {
		return
			Commands.defer(
				() -> this.moveToPosePID(() -> FieldConstants.Reef.getTranslatedPose(
					this.getTargetAlgae(),
					distance.get())),
				Set.of(this));
	}

	/**
	 * Field relative drive command using two joysticks (controlling linear and
	 * angular velocities).
	 */
	public Command driveWithAngleSetpoint(
			DoubleSupplier xSupplier,
			DoubleSupplier ySupplier,
			Supplier<Rotation2d> rotation) {
		return new FunctionalCommand(
			() -> {
				this.isHeadingPIDEnabled = true;
				this.headingPID.reset(this.getPose().getRotation().getRadians());
			},
			() -> {
				this.headingPID.setGoal(rotation.get().getRadians());
				double x = xSupplier.getAsDouble();
				double y = ySupplier.getAsDouble();

				// linear
				double linearSpeedScalar;
				linearSpeedScalar = Math.hypot(x, y);

				linearSpeedScalar = MathUtil.clamp(linearSpeedScalar, -1.0, 1.0);
				linearSpeedScalar = MathUtil.applyDeadband(linearSpeedScalar, DriveConstants.JOYSTICK_DEADBAND_LINEAR);


				linearSpeedScalar = CometMathUtil.minMaxScale(
					linearSpeedScalar,
					DriveConstants.JOYSTICK_DEADBAND_LINEAR,
					1
				);

				Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));
				Translation2d linearTranslation = new Translation2d(linearSpeedScalar, linearDirection);
				


				ChassisSpeeds speeds = new ChassisSpeeds(
						this.getMaximumSpeed().times(linearTranslation.getX()),
						this.getMaximumSpeed().times(linearTranslation.getY()),
						RadiansPerSecond.of(0));

				boolean isFlipped = DriverStation.getAlliance().isPresent()
						&& DriverStation.getAlliance().get() == Alliance.Red;

				Rotation2d robotAngle = isFlipped
						? this.getRotation().plus(new Rotation2d(Degrees.of(180)))
						: this.getRotation();

				speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, robotAngle);

				this.runVelocity(speeds);
			},
		(interrupted) -> this.isHeadingPIDEnabled = false,
		() -> false,
		this
		);
	}

	/**
	 * Field relative drive command using two joysticks (controlling linear and
	 * angular velocities).
	 */
	public Command driveWithXandAngleSetpoint(
			DoubleSupplier xSupplier,
			DoubleSupplier ySupplier,
			Supplier<Rotation2d> rotation) {
		return new FunctionalCommand(
			() -> {
				this.isHeadingPIDEnabled = true;
				this.headingPID.reset(this.getPose().getRotation().getRadians());
				this.xPID.reset();
			},
			() -> {
				this.headingPID.setGoal(rotation.get().getRadians());
				this.xPID.setSetpoint(xSupplier.getAsDouble());

				// linear
				double ySpeedScalar;

				ySpeedScalar = ySupplier.getAsDouble();
				ySpeedScalar = MathUtil.clamp(ySpeedScalar, -1.0, 1.0);
				ySpeedScalar = MathUtil.applyDeadband(ySpeedScalar, DriveConstants.JOYSTICK_DEADBAND_LINEAR);
				ySpeedScalar = CometMathUtil.minMaxScale(
					ySpeedScalar,
					DriveConstants.JOYSTICK_DEADBAND_LINEAR,
					1
				);

				ChassisSpeeds speeds = new ChassisSpeeds(
					MetersPerSecond.zero(),
					this.getMaximumSpeed().times(ySpeedScalar),
					RadiansPerSecond.zero()
				);

				boolean isFlipped = DriverStation.getAlliance().isPresent()
						&& DriverStation.getAlliance().get() == Alliance.Red;

				Rotation2d robotAngle = isFlipped
						? this.getRotation().plus(new Rotation2d(Degrees.of(180)))
						: this.getRotation();

				speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, robotAngle);

				this.runVelocity(speeds);
			},
		(interrupted) -> {
			this.isHeadingPIDEnabled = false;
			this.isXPIDEnabled = false;
		},
		() -> headingPID.atGoal() && xPID.atSetpoint(),
		this
		);
	}


	/**
	 * Turns the robot to a specified angle using PID control.
	 * To specify both translation and rotation, use
	 * {@link #moveToPosePID(Supplier)}.
	 */
	public Command turnToAngle(Supplier<Rotation2d> rotation) {
		return new FunctionalCommand(
				() -> {
					this.isHeadingPIDEnabled = true;
					this.headingPID.reset(this.getPose().getRotation().getRadians());
				},
				() -> this.headingPID.setGoal(rotation.get().getRadians()),
				(interrupted) -> this.isHeadingPIDEnabled = false,
				this.headingPID::atSetpoint,
				this);
	}

	/**
	 * Moves the robot to a specified translation using PID control.
	 * To specify both translation and rotation, use
	 * {@link #moveToPosePID(Supplier)}.
	 */
	public Command moveToTranslationPID(Supplier<Translation2d> pose) {
		return new FunctionalCommand(
				() -> {
					this.isXPIDEnabled = true;
					this.isYPIDEnabled = true;
					this.xPID.reset();
					this.yPID.reset();
				},
				() -> {
					this.xPID.setSetpoint(pose.get().getX());
					this.yPID.setSetpoint(pose.get().getY());
				},
				(interrupted) -> {
					this.isXPIDEnabled = false;
					this.isYPIDEnabled = false;
				},
				() -> this.xPID.atSetpoint() && this.yPID.atSetpoint(),
				this);
	}

	/**
	 * Moves the robot to a specified pose using PID control.
	 * 
	 * @param pose the target pose
	 * @return the command
	 */
	public Command moveToPosePID(Supplier<Pose2d> pose) {
		return new FunctionalCommand(
				() -> {
					this.isHeadingPIDEnabled = true;
					this.isXPIDEnabled = true;
					this.isYPIDEnabled = true;
					// we do this just so that atSetpoint() doesn't return true immediately
					this.headingPID.reset(this.getPose().getRotation().getRadians());
					this.xPID.reset();
					this.yPID.reset();
				},
				() -> {
					this.headingPID.setGoal(pose.get().getRotation().getRadians());
					this.xPID.setSetpoint(pose.get().getX());
					this.yPID.setSetpoint(pose.get().getY());
				},
				(interrupted) -> {
					this.isHeadingPIDEnabled = false;
					this.isXPIDEnabled = false;
					this.isYPIDEnabled = false;
					this.stop();
				},
				() -> this.headingPID.atSetpoint() && this.xPID.atSetpoint() && this.yPID.atSetpoint(),
				this);
	}

	public Command sysIdLinear() {
		SysIdRoutine routine = new SysIdRoutine(
			new SysIdRoutine.Config(
				Volts.per(Second).of(0.5),
				Volts.of(2),
				null,
				(state) -> Logger.recordOutput(
					"SysId/drive-linear", state.toString()
				)
			),
			new SysIdRoutine.Mechanism(
				this::runDriveVoltage,
				log -> {
					Logger.recordOutput("SysId/drive-linear/Voltage", this.getAverageModuleDriveAppliedVoltage());
					Logger.recordOutput("SysId/drive-linear/Position", this.getAverageDriveAngularPosition());
					Logger.recordOutput("SysId/drive-linear/Velocity", this.getAverageDriveAngularVelocity());
					log.motor("drive-linear")
						.voltage(this.getAverageModuleDriveAppliedVoltage())
						.angularPosition(this.getAverageDriveAngularPosition())
						.angularVelocity(this.getAverageDriveAngularVelocity());
				}, 
				this)
		);


		Command routineCommand =
			Commands.parallel(
				Commands.run(() -> this.setDriveAngleSetpoints(new Rotation2d())),
				Commands.sequence(
					routine.dynamic(Direction.kForward),
					Commands.waitSeconds(2),
					routine.dynamic(Direction.kReverse),
					Commands.waitSeconds(2),
					routine.quasistatic(Direction.kForward),
					Commands.waitSeconds(2),
					routine.quasistatic(Direction.kReverse)
				)
			);
		return routineCommand;
	}


	public Command sysIdAngular() {
		SysIdRoutine routine = new SysIdRoutine(
			new SysIdRoutine.Config(
				Volts.per(Second).of(0.5),
				Volts.of(3),
				null,
				(state) -> Logger.recordOutput(
					"SysId/drive-rotational", state.toString()
				)
			),
			new SysIdRoutine.Mechanism(
				this::runDriveVoltage,
				log -> {
					Logger.recordOutput("SysId/drive-rotational/Voltage", this.getAverageModuleDriveAppliedVoltage());
					Logger.recordOutput("SysId/drive-rotational/Position", this.getAverageDriveAngularPosition());
					Logger.recordOutput("SysId/drive-rotational/Velocity", this.getAverageDriveAngularVelocity());
					log.motor("drive-rotational")
						.voltage(this.getAverageModuleDriveAppliedVoltage())
						.angularPosition(this.getAverageDriveAngularPosition())
						.angularVelocity(this.getAverageDriveAngularVelocity());
				}, 
				this)
		);


		Command routineCommand =
			Commands.parallel(
				Commands.run(() -> this.setDriveAngleSetpointToRotationPattern()),
				Commands.sequence(
					routine.dynamic(Direction.kForward),
					Commands.waitSeconds(2),
					routine.dynamic(Direction.kReverse),
					Commands.waitSeconds(2),
					routine.quasistatic(Direction.kReverse),
					Commands.waitSeconds(2),
					routine.quasistatic(Direction.kForward)
				)
			);
		return routineCommand;
	}
}
