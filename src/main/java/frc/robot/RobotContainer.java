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

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Volts;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeAlgaeOnField;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.AutoScoreProcessor;
import frc.robot.commands.ShootAtTargetFromDistance;
import frc.robot.commands.ShootOnMove;
import frc.robot.commands.coral.AutoDepositCoralD;
import frc.robot.commands.coral.AutoDepositCoralE;
import frc.robot.commands.hoodtake.HoodtakeFromReef;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.gyro.GyroIO;
import frc.robot.subsystems.drive.gyro.GyroIOPigeon2;
import frc.robot.subsystems.drive.gyro.GyroIOSim;
import frc.robot.subsystems.drive.module.ModuleIO;
import frc.robot.subsystems.drive.module.ModuleIOMapleSim;
import frc.robot.subsystems.drive.module.ModuleIOSpark;
import frc.robot.subsystems.hoodtake.Hoodtake;
import frc.robot.subsystems.hoodtake.HoodtakeConstants;
import frc.robot.subsystems.hoodtake.HoodtakeIO;
import frc.robot.subsystems.hoodtake.HoodtakeIOSim;
import frc.robot.subsystems.hoodtake.HoodtakeIOSpark;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.indexer.IndexerIOSpark;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOSpark;
import frc.robot.subsystems.shooter.NetTargetSelector;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOSpark;
import frc.robot.subsystems.vision.VisionConstants.Camera;
import frc.robot.subsystems.vision.apriltag.ApriltagVision;
import frc.robot.subsystems.vision.apriltag.ApriltagVisionIO;
import frc.robot.subsystems.vision.apriltag.ApriltagVisionIOPhotonVision;
import frc.robot.util.AllianceColor;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.controller.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
	// Subsystems
	private final Drive drive;
	private final ApriltagVision vision;
	private final Shooter shooter;
	private final Intake intake;
	private final Hoodtake hoodtake;
	private final Indexer indexer;

	private final CometController controller = new CometPS4Controller(0);

	private final LoggedDashboardChooser<Command> autoChooser;
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    
	private SwerveDriveSimulation swerveDriveSimulation;



	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		switch (Constants.currentMode) {
			case REAL: // Real robot, instantiate hardware IO implementations
				this.drive = new Drive(
						new GyroIOPigeon2(),
						new ModuleIOSpark(0),
						new ModuleIOSpark(1),
						new ModuleIOSpark(2),
						new ModuleIOSpark(3));
				this.vision = new ApriltagVision(drive::addVisionMeasurement,
						new ApriltagVisionIOPhotonVision(Camera.FrontApriltag), new ApriltagVisionIOPhotonVision(Camera.BackApriltag));
				this.shooter = new Shooter(new ShooterIOSpark());
				this.intake = new Intake(new IntakeIOSpark());
				this.hoodtake = new Hoodtake(new HoodtakeIOSpark());
				this.indexer = new Indexer(new IndexerIOSpark());
				break;

			case SIM:
				DriveTrainSimulationConfig driveTrainSimulationConfig = DriveTrainSimulationConfig.Default()
						.withGyro(COTS.ofPigeon2())
						.withSwerveModule(
								COTS.ofMark4i(DCMotor.getNEO(1), DCMotor.getNEO(1), COTS.WHEELS.COLSONS.cof, 1))
						.withTrackLengthTrackWidth(Meters.of(0.3429), Meters.of(0.4953))
						.withBumperSize(Meters.of(0.84), Meters.of(0.87));

				this.swerveDriveSimulation = new SwerveDriveSimulation(
						driveTrainSimulationConfig, // Specify Configuration
						new Pose2d(0, 0, new Rotation2d(0)) // Specify starting pose
				);

				this.drive = new Drive(
					new GyroIOSim(this.swerveDriveSimulation.getGyroSimulation()),
					new ModuleIOMapleSim(this.swerveDriveSimulation.getModules()[0]),
					new ModuleIOMapleSim(this.swerveDriveSimulation.getModules()[1]),
					new ModuleIOMapleSim(this.swerveDriveSimulation.getModules()[2]),
					new ModuleIOMapleSim(this.swerveDriveSimulation.getModules()[3]));

				this.swerveDriveSimulation.setSimulationWorldPose(this.drive.getPose());

				// Register the drivetrain simulation to the default simulation world
				SimulatedArena.getInstance().addDriveTrainSimulation(swerveDriveSimulation);
				SimulatedArena.getInstance().addGamePiece(new ReefscapeAlgaeOnField(new Translation2d(2,2)));
				SimulatedArena.getInstance().addGamePiece(new ReefscapeAlgaeOnField(new Translation2d(2.1,2)));
				SimulatedArena.getInstance().addGamePiece(new ReefscapeAlgaeOnField(new Translation2d(2.2,2)));
				SimulatedArena.getInstance().addGamePiece(new ReefscapeAlgaeOnField(new Translation2d(2.3,2)));
				SimulatedArena.getInstance().addGamePiece(new ReefscapeAlgaeOnField(new Translation2d(2.4,2)));


				// APRILTAG VISION SIM IS TOO COMPUTATIONALLY INTENSIVE

				/*this.vision = new ApriltagVision(
						drive::addVisionMeasurement,
						new ApriltagVisionIOPhotonVisionSim(
								"limelight-shooter",
								robotToCamera0,
								swerveDriveSimulation::getSimulatedDriveTrainPose));*/
				this.vision = new ApriltagVision(drive::addVisionMeasurement, new ApriltagVisionIO() {});
				this.shooter = new Shooter(new ShooterIOSim());
				this.intake = new Intake(new IntakeIOSim());
				this.hoodtake = new Hoodtake(new HoodtakeIOSim());
				this.indexer = new Indexer(new IndexerIOSim());
				break;

			default: // Replayed robot, disable IO implementations
				this.drive = new Drive(
					new GyroIO() {},
					new ModuleIO() {},
					new ModuleIO() {},
					new ModuleIO() {},
					new ModuleIO() {}
				);
				this.vision = new ApriltagVision(drive::addVisionMeasurement, new ApriltagVisionIO() {});
				this.shooter = new Shooter(new ShooterIO() {});
				this.intake = new Intake(new IntakeIO() {});
				this.hoodtake = new Hoodtake(new HoodtakeIO() {});
				this.indexer = new Indexer(new IndexerIO() {});
				break;
		}

		setupAutoCommands();
		this.autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
		setupDefaultCommands();
		setupButtonBindings();

		DriverStation.silenceJoystickConnectionWarning(true);

		SmartDashboard.putNumber("Shooter/topSpeedRPM", 750);
		SmartDashboard.putNumber("Shooter/botSpeedRPM", 1500);
		SmartDashboard.putNumber("Shooter/topP", ShooterConstants.TOP_WHEEL_kP);
		SmartDashboard.putNumber("Shooter/botP", ShooterConstants.BOT_WHEEL_kP);
		SmartDashboard.putNumber("Shooter/topI", ShooterConstants.TOP_WHEEL_kI);
		SmartDashboard.putNumber("Shooter/botI", ShooterConstants.BOT_WHEEL_kI);
		SmartDashboard.putNumber("Shooter/topD", ShooterConstants.TOP_WHEEL_kD);
		SmartDashboard.putNumber("Shooter/botD", ShooterConstants.BOT_WHEEL_kD);
	}

	private void setupAutoCommands() {
		NamedCommands.registerCommand("Hoodtake From Reef", new HoodtakeFromReef(drive, hoodtake));
		NamedCommands.registerCommand("Deposit Coral E", new AutoDepositCoralE(drive, intake));
		NamedCommands.registerCommand("Deposit Coral D", new AutoDepositCoralD(drive, intake));
	}

	private void setupDefaultCommands() {
		this.drive.setDefaultCommand(
			this.drive.joystickDrive(
				() -> -controller.getLeftY(),
				() -> -controller.getLeftX(),
				/*() ->  {
					int left = controller.leftBumper().getAsBoolean() ? 1 : 0;
					int right = controller.rightBumper().getAsBoolean() ? 1 : 0;
					return right-left;
				}*/
				() -> -controller.getRightX()
			)
		);

		this.shooter.setDefaultCommand(
			this.shooter.setBottomVoltage(() -> Volts.of(0))
			.andThen(this.shooter.setTopVoltage(() -> Volts.of(0)))
		);

		this.hoodtake.setDefaultCommand(
			Commands.sequence(
				this.hoodtake.setPivotPositionSetpoint(() -> HoodtakeConstants.STARTING_ANGLE),
				Commands.either(
					this.hoodtake.setPivotVoltage(() -> Volts.of(0)),
					this.hoodtake.setPivotPosition(() -> HoodtakeConstants.STARTING_ANGLE),
					this.hoodtake::atPosition
				),
				this.hoodtake.setWheelVoltage(() -> Volts.of(0))
			)
		);

		this.indexer.setDefaultCommand(
			Commands.sequence(
				// this.indexer.setLeftVoltage(() -> Volts.of(0)),
				// this.indexer.setRightVoltage(() -> Volts.of(0))
				// ,
				
				Commands.either(
					this.indexer.setLeftVoltage(() -> Volts.of(0)),
					this.indexer.setLeftVoltage(() -> Volts.of(-1)),
					() -> indexer.getLeftPosition().lt(Degrees.of(40))
				),
				
				Commands.either(
					this.indexer.setRightVoltage(() -> Volts.of(0)),
					this.indexer.setRightVoltage(() -> Volts.of(-1)),
					() -> indexer.getRightPosition().lt(Degrees.of(40))
				)
			)
		);

		this.intake.setDefaultCommand(
			Commands.sequence(
				Commands.either(
					this.intake.setPivotVoltage(() -> Volts.of(0.3)),
					this.intake.setPivotPosition(() -> IntakeConstants.STARTING_ANGLE),
					() -> intake.getPivotPosition().gt(Degrees.of(85))
				),
				this.intake.setWheelVoltage(() -> Volts.of(0))
			)
		);
	}

	private void setupButtonBindings() {
		// Lock to 0° when A button is held
		/*
		 * controller
		 * .a()
		 * .whileTrue(
		 * DriveCommands.joystickDriveAtAngle(
		 * drive,
		 * () -> -controller.getLeftY(),
		 * () -> -controller.getLeftX(),
		 * () -> new Rotation2d()));
		 */

		// Switch to cross pattern when options button is pressed
		// controller.rightMenu().onTrue(Commands.runOnce(drive::stopWithX, drive));

		// Reset gyro to 0° when A button is pressed
		controller.a().onTrue(Commands.runOnce(() -> drive.resetHeadingWithAlliance(), drive)
			.ignoringDisable(true));

		this.controller.leftTrigger().whileTrue(
			Commands.sequence(
				this.hoodtake.setPivotPosition(() -> Degrees.of(90)),
				this.shooter.setFlywheelVelocities(
					() -> RPM.of(SmartDashboard.getNumber("Shooter/topSpeedRPM", 2000)),
					() -> RPM.of(SmartDashboard.getNumber("Shooter/botSpeedRPM", 2000))
				),
				// this.shooter.setBottomVoltage(() -> Volts.of(3)),
				Commands.waitUntil(() -> false)
			)
		);

		// this.controller.x().whileTrue(
		// 	new ShootAtTargetFromDistance(controller, drive, shooter, hoodtake)
		// );

		this.controller.rightTrigger().whileTrue(
			Commands.sequence(
				this.indexer.shoot()
				// Commands.waitUntil(() -> false)
			)
		);

		// intake
		this.controller.leftBumper().whileTrue(
			Commands.sequence(
				this.intake.setWheelVoltage(() -> Volts.of(3.5)),
				this.intake.setPivotPosition(() -> Degrees.of(35)),
				Commands.waitUntil(() -> false)
			)
		);

		// processor
		this.controller.rightBumper().whileTrue(
			Commands.sequence(
				this.intake.setWheelVoltage(() -> Volts.of(-3)),
				this.intake.setPivotPosition(() -> Degrees.of(85)),
				this.indexer.setRightVoltage(() -> Volts.of(3)),
				Commands.waitUntil(() -> false)
			)
		);

		// L3 Hoodtake

		this.controller.y().whileTrue(
			Commands.sequence(
				this.hoodtake.setPivotPosition(() -> Degrees.of(55)),
				this.hoodtake.setWheelVoltage(() -> Volts.of(-5)),
				Commands.waitUntil(() -> false)
			)
		);

		//L2

		this.controller.b().whileTrue(
			Commands.sequence(
				this.hoodtake.setPivotPosition(() -> Degrees.of(45)),
				this.hoodtake.setWheelVoltage(() -> Volts.of(5)),
				Commands.waitUntil(() -> false)
			)
		);

		this.controller.left().whileTrue(
			Commands.sequence(
				this.intake.setPivotPosition(() -> Degrees.of(60)),
				Commands.waitUntil(() -> intake.atPosition()),
				this.intake.setWheelVoltage(() -> Volts.of(-7)),
				Commands.waitUntil(() -> false)
			)
		);

		// this.controller.left().whileTrue(
		// 	new AutoDepositCoral(drive, intake)
		// );

		// this.controller.right().whileTrue(
		// 	this.indexer.setRightVoltage(() -> Volts.of(3))
		// 	.andThen(Commands.waitUntil(() -> false))
		// );
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		return autoChooser.get();
	}

	/**
	 * Basically since I disabled vision sim the pose estimator never receives any vision data,
	 * and so the robot's actual position and its estimated pose kinda just drift apart
	 * more and more. This is a temporary fix which is run every loop during sim.
	 * 
	 * This should only be used when vision sim is disabled
	 */
	public void updateSimDrivePosition() {
		this.drive.addVisionMeasurement(
			this.swerveDriveSimulation.getSimulatedDriveTrainPose(),
		 	Timer.getFPGATimestamp(),
			VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(1))
		);
	}

    public void updateSimulation() {
        if (Constants.currentMode != Constants.Mode.SIM) return;

        Logger.recordOutput("FieldSimulation/RobotPosition", swerveDriveSimulation.getSimulatedDriveTrainPose());
        Logger.recordOutput(
                "FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
        Logger.recordOutput(
                "FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
    }
}
