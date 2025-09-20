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
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Volts;

import java.util.Set;

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
import frc.robot.Constants.Mode;
import frc.robot.commands.AutonCommandBuilder;
import frc.robot.commands.ScoreProcessor;
import frc.robot.commands.coral.AutoDepositCoralD;
import frc.robot.commands.coral.AutoDepositCoralE;
import frc.robot.commands.hoodtake.HoodtakeFromReef;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.SwerveConstants;
import frc.robot.subsystems.drive.gyro.GyroIO;
import frc.robot.subsystems.drive.gyro.GyroIOPigeon2;
import frc.robot.subsystems.drive.gyro.GyroIOSim;
import frc.robot.subsystems.drive.module.ModuleIO;
import frc.robot.subsystems.drive.module.ModuleIOMapleSim;
import frc.robot.subsystems.drive.module.ModuleIOSpark;
import frc.robot.subsystems.hoodtake.Hoodtake;
import frc.robot.subsystems.hoodtake.HoodtakeIO;
import frc.robot.subsystems.hoodtake.HoodtakeIOSim;
import frc.robot.subsystems.hoodtake.HoodtakeIOSpark;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.indexer.IndexerIOSpark;
import frc.robot.subsystems.intakeLeft.LeftIntake;
import frc.robot.subsystems.intakeLeft.*;
import frc.robot.subsystems.intakeLeft.LeftIntakeIO;
import frc.robot.subsystems.intakeLeft.LeftIntakeIOSim;
import frc.robot.subsystems.intakeLeft.LeftIntakeIOSpark;
import frc.robot.subsystems.intakeRight.RightIntake;
import frc.robot.subsystems.intakeRight.RightIntakeConstants;
import frc.robot.subsystems.intakeRight.RightIntakeIO;
import frc.robot.subsystems.intakeRight.RightIntakeIOSim;
import frc.robot.subsystems.intakeRight.RightIntakeIOSpark;
import frc.robot.subsystems.vision.VisionConstants.Camera;
import frc.robot.subsystems.vision.apriltag.ApriltagVision;
import frc.robot.subsystems.vision.apriltag.ApriltagVisionIO;
import frc.robot.subsystems.vision.apriltag.ApriltagVisionIOPhotonVision;
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
	private final LeftIntake leftIntake;
	private final RightIntake rightIntake;
	private final Hoodtake hoodtake;
	private final Indexer indexer;

	private final CometController controller = new CometXboxController(0);
	private final CometController backupController = new CometPS4Controller(1);

	private final LoggedDashboardChooser<Command> autoChooser;
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    
	private SwerveDriveSimulation swerveDriveSimulation;

	private AutonCommandBuilder autonCommandBuilder = new AutonCommandBuilder();



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
				this.leftIntake = new LeftIntake(new LeftIntakeIOSpark());
				this.rightIntake = new RightIntake(new RightIntakeIOSpark());
				this.hoodtake = new Hoodtake(new HoodtakeIOSpark());
				this.indexer = new Indexer(new IndexerIOSpark());
				break;

			case SIM:
				DriveTrainSimulationConfig driveTrainSimulationConfig = DriveTrainSimulationConfig.Default()
						.withGyro(COTS.ofPigeon2())
						.withSwerveModule(
								COTS.ofMark4i(DCMotor.getNEO(1), DCMotor.getNEO(1), COTS.WHEELS.COLSONS.cof, 1))
						.withTrackLengthTrackWidth(SwerveConstants.WHEELBASE, SwerveConstants.TRACK_WIDTH)
						.withBumperSize(Meters.of(0.876), Meters.of(0.876));

				this.swerveDriveSimulation = new SwerveDriveSimulation(
						driveTrainSimulationConfig, // Specify Configuration
						Pose2d.kZero // Specify starting pose
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
				this.leftIntake = new LeftIntake(new LeftIntakeIOSim());
				this.rightIntake = new RightIntake(new RightIntakeIOSim());
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
				this.vision = new ApriltagVision(drive::addVisionMeasurement, new ApriltagVisionIO() {}, new ApriltagVisionIO() {});
				this.leftIntake = new LeftIntake(new LeftIntakeIO() {});
				this.rightIntake = new RightIntake(new RightIntakeIO() {});
				this.hoodtake = new Hoodtake(new HoodtakeIO() {});
				this.indexer = new Indexer(new IndexerIO() {});
				break;
		}

		setupAutoCommands();
		this.autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
		setupDefaultCommands();
		setupButtonBindings();

		DriverStation.silenceJoystickConnectionWarning(true);

		SmartDashboard.putNumber("Intake/RightIntakingVolts", 5);
		SmartDashboard.putNumber("Intake/RightIntakeAngle", 30);
		SmartDashboard.putNumber("Intake/LeftIntakingVolts", 5);
		SmartDashboard.putNumber("Intake/LeftIntakeAngle", 30);

	}

	private void setupAutoCommands() {
		// NamedCommands.registerCommand("Hoodtake From Reef", new HoodtakeFromReef(drive, hoodtake, drive::getTargetAlgae));
		NamedCommands.registerCommand("Deposit Coral E", new AutoDepositCoralE(drive, leftIntake));
		NamedCommands.registerCommand("Deposit Coral D", new AutoDepositCoralD(drive, leftIntake));
		
	
		NamedCommands.registerCommand("AutonBuilder", Commands.defer(() -> autonCommandBuilder.getCommand(drive, hoodtake), Set.of(drive, hoodtake)));
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


		this.hoodtake.setDefaultCommand(
			this.hoodtake.defaultCommand()
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

		this.leftIntake.setDefaultCommand(
			Commands.sequence(
				Commands.either(
					this.leftIntake.setPivotVoltage(() -> Volts.of(0.2)),
					this.leftIntake.setPivotPosition(() -> LeftIntakeConstants.STARTING_ANGLE),
					() -> leftIntake.getPivotPosition().gt(Degrees.of(85))
				),
				this.leftIntake.setWheelVoltage(() -> Volts.of(0))
			)
		);

		this.rightIntake.setDefaultCommand(
			Commands.sequence(
				Commands.either(
					this.rightIntake.setPivotVoltage(() -> Volts.of(0.2)),
					this.rightIntake.setPivotPosition(() -> RightIntakeConstants.STARTING_ANGLE),
					() -> rightIntake.getPivotPosition().gt(Degrees.of(85))
				),
				this.rightIntake.setWheelVoltage(() -> Volts.of(0))
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


		// Reset gyro to 0° when A button is pressed
		controller.a().onTrue(Commands.runOnce(() -> drive.resetHeadingWithAlliance(), drive)
			.ignoringDisable(true));

		// this.controller.leftTrigger().toggleOnTrue(
		// 	Commands.sequence(
		// 		this.hoodtake.setPivotPosition(() -> Degrees.of(90)),
		// 		this.shooter.setFlywheelVelocities(
		// 			() -> RPM.of(SmartDashboard.getNumber("Shooter/topSpeedRPM", 2000)),
		// 			() -> RPM.of(SmartDashboard.getNumber("Shooter/botSpeedRPM", 2000))
		// 		),
		// 		// this.shooter.setTopVoltage(() -> Volts.of(6)),
		// 		// this.shooter.setBottomVoltage(() -> Volts.of(3)),
		// 		Commands.waitUntil(() -> false)
		// 	)
		// );

		// left intake
		this.controller.leftBumper().whileTrue(
			Commands.sequence(
				this.leftIntake.setWheelVoltage(() -> Volts.of(SmartDashboard.getNumber("Intake/LeftIntakingVolts", 5))),
				// this.intake.setWheelVelocity(() -> RPM.of(500)),
				this.leftIntake.setPivotPosition(() -> Degrees.of((SmartDashboard.getNumber("Intake/LeftIntakeAngle", 30)))),
				Commands.waitUntil(() -> false)
			)
		);

		// right intake
		this.controller.rightBumper().whileTrue(
			Commands.sequence(
				this.rightIntake.setWheelVoltage(() -> Volts.of(SmartDashboard.getNumber("Intake/RightIntakingVolts", 5))),
				// this.intake.setWheelVelocity(() -> RPM.of(500)),
				this.rightIntake.setPivotPosition(() -> Degrees.of((SmartDashboard.getNumber("Intake/RightIntakeAngle", 30)))),
				Commands.waitUntil(() -> false)
			)
		);

		// left processor
		this.controller.leftTrigger().whileTrue(
			Commands.sequence(
				new ScoreProcessor(leftIntake, indexer),
				Commands.waitUntil(() -> false)
			)
		);

		// right processor
		this.controller.rightTrigger().whileTrue(
			Commands.sequence(
				new ScoreProcessor(rightIntake, indexer),
				Commands.waitUntil(() -> false)
			)
		);

		// L3 Hoodtake

		this.controller.y().whileTrue(
			Commands.sequence(
				this.hoodtake.setPivotPosition(() -> Degrees.of(55)),
				this.hoodtake.setWheelVoltage(() -> Volts.of(-6)),
				Commands.waitUntil(() -> false)
			)
		);

		//L2

		this.controller.b().whileTrue(
			Commands.sequence(
				this.hoodtake.setPivotPosition(() -> Degrees.of(44)),
				this.hoodtake.setWheelVoltage(() -> Volts.of(6)),
				Commands.waitUntil(() -> false)
			)
		);

		// // score l1 coral
		// this.controller.left().whileTrue(
		// 	Commands.sequence(
		// 		this.intake.setPivotPosition(() -> Degrees.of(60)),
		// 		Commands.waitUntil(() -> intake.atPosition()),
		// 		this.intake.setWheelVoltage(() -> Volts.of(-7)),
		// 		Commands.waitUntil(() -> false)
		// 	)
		// );

		this.controller.right().whileTrue(
			Commands.defer(
				() -> new HoodtakeFromReef(drive, hoodtake, drive::getTargetAlgae),
				Set.of(drive, hoodtake)
			)
			.andThen(Commands.waitUntil(() -> false))
		);

		// this.controller.up().whileTrue(
		// 	Commands.sequence(
		// 		this.intake.setPivotPosition(() -> Degrees.of(50)),
		// 		Commands.waitSeconds(2),
		// 		this.intake.sysIdRoutineWheel()
		// 	)
			
		// );

		// this.controller.down().whileTrue(
		// 	this.intake.sysIdRoutinePivot()
		// );

		// Reset gyro to 0° when A button is pressed
		backupController.a().onTrue(Commands.runOnce(() -> drive.resetHeadingWithAlliance(), drive)
			.ignoringDisable(true));


		this.backupController.rightTrigger().whileTrue(
			Commands.sequence(
				this.indexer.shoot()
				// Commands.waitUntil(() -> false)
			)
		);

		// L3 Hoodtake

		this.backupController.y().whileTrue(
			Commands.sequence(
				this.hoodtake.setPivotPosition(() -> Degrees.of(55)),
				this.hoodtake.setWheelVoltage(() -> Volts.of(-6)),
				Commands.waitUntil(() -> false)
			)
		);

		//L2

		this.backupController.b().whileTrue(
			Commands.sequence(
				this.hoodtake.setPivotPosition(() -> Degrees.of(45)),
				this.hoodtake.setWheelVoltage(() -> Volts.of(6)),
				Commands.waitUntil(() -> false)
			)
		);

		// this.backupController.left().whileTrue(
		// 	Commands.sequence(
		// 		this.intake.setPivotPosition(() -> Degrees.of(60)),
		// 		Commands.waitUntil(() -> intake.atPosition()),
		// 		this.intake.setWheelVoltage(() -> Volts.of(-7)),
		// 		Commands.waitUntil(() -> false)
		// 	)
		// );

		this.backupController.up().whileTrue(
			Commands.sequence(
				this.indexer.setRightVoltage(() -> Volts.of(-1))
			)
			.andThen(Commands.waitUntil(() -> false))
		);
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
		if (Constants.simMode == Mode.REPLAY)
			return;
			
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

	public void enabledInit() {
		this.leftIntake.enabledInit();
		this.rightIntake.enabledInit();
		this.hoodtake.enabledInit();
	}
}
