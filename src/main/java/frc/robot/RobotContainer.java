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

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera0;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ShootOnMove;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.gyro.GyroIO;
import frc.robot.subsystems.drive.gyro.GyroIOPigeon2;
import frc.robot.subsystems.drive.gyro.GyroIOSim;
import frc.robot.subsystems.drive.module.ModuleIO;
import frc.robot.subsystems.drive.module.ModuleIOMapleSim;
import frc.robot.subsystems.drive.module.ModuleIOSpark;
import frc.robot.subsystems.hoodtake.Hoodtake;
import frc.robot.subsystems.hoodtake.HoodtakeIOSim;
import frc.robot.subsystems.hoodtake.HoodtakeIOSpark;
import frc.robot.subsystems.hoodtake.HoodtakeIO;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOSpark;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOSpark;
import frc.robot.subsystems.vision.VisionConstants.Camera;
import frc.robot.subsystems.vision.apriltag.ApriltagVision;
import frc.robot.subsystems.vision.apriltag.ApriltagVisionIO;
import frc.robot.subsystems.vision.apriltag.ApriltagVisionIOPhotonVisionSim;
import frc.robot.util.controller.*;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import frc.robot.subsystems.vision.apriltag.ApriltagVisionIOPhotonVision;

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

	private final CometController controller = new CometLogitechController(0);

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
						new Pose2d(3, 3, new Rotation2d()) // Specify starting pose
				);
				// Register the drivetrain simulation to the default simulation world
				SimulatedArena.getInstance().addDriveTrainSimulation(swerveDriveSimulation);

				this.drive = new Drive(
						new GyroIOSim(this.swerveDriveSimulation.getGyroSimulation()),
						new ModuleIOMapleSim(this.swerveDriveSimulation.getModules()[0]),
						new ModuleIOMapleSim(this.swerveDriveSimulation.getModules()[1]),
						new ModuleIOMapleSim(this.swerveDriveSimulation.getModules()[2]),
						new ModuleIOMapleSim(this.swerveDriveSimulation.getModules()[3]));
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
				break;
		}

		this.autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
		setupAutoRoutines();
		setupButtonBindings();

		DriverStation.silenceJoystickConnectionWarning(true);
	}

	private void setupAutoRoutines() {
		// Set up auto routines

		// Set up SysId routines
		/*
		 * autoChooser.addOption(
		 * "Drive Wheel Radius Characterization",
		 * DriveCommands.wheelRadiusCharacterization(drive));
		 * autoChooser.addOption(
		 * "Drive Simple FF Characterization",
		 * DriveCommands.feedforwardCharacterization(drive));
		 * autoChooser.addOption(
		 * "Drive SysId (Quasistatic Forward)",
		 * drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
		 * autoChooser.addOption(
		 * "Drive SysId (Quasistatic Reverse)",
		 * drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
		 * autoChooser.addOption(
		 * "Drive SysId (Dynamic Forward)",
		 * drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
		 * autoChooser.addOption(
		 * "Drive SysId (Dynamic Reverse)",
		 * drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
		 */
	}

	private void setupButtonBindings() {
		this.drive.setDefaultCommand(
			this.drive.joystickDrive(
				() -> -controller.getLeftY(),
				() -> -controller.getLeftX(),
				() ->  {
					int left = controller.leftBumper().getAsBoolean() ? 1 : 0;
					int right = controller.rightBumper().getAsBoolean() ? 1 : 0;
					return right-left;
				}
			)
		);


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
		controller.rightMenu().onTrue(Commands.runOnce(drive::stopWithX, drive));

		// Reset gyro to 0° when A button is pressed
		controller
			.a()
			.onTrue(
				Commands.runOnce(
					() -> {
						drive.resetHeadingWithAlliance();
					},
				drive
				)
				.ignoringDisable(true));

		this.controller.x().whileTrue(
			this.intake.setPosition(() -> Degrees.of(25))
		);

		this.controller.y().whileTrue(
			this.intake.setPosition(() -> Degrees.of(90))
		);

		/*this.controller.b().whileTrue(
			DriveCommands.feedforwardCharacterization(drive)
		);
		this.controller.b().whileTrue(DriveCommands.feedforwardCharacterization(drive));
		*/

		this.controller.left().whileTrue(
			this.drive.turnToAngle(() -> new Rotation2d(Degrees.of(90)))
		);

		this.controller.right().whileTrue(
			this.intake.setWheelVelocity(() -> RPM.of(50))	
		);

		//this.controller.b().whileTrue(this.intake.sysIdRoutineWheel());
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		return autoChooser.get();
	}

	public void displaySimFieldToAdvantageScope() {
		if (Constants.currentMode != Constants.Mode.SIM)
			return;

		Logger.recordOutput(
				"FieldSimulation/RobotPosition", swerveDriveSimulation.getSimulatedDriveTrainPose());
		Logger.recordOutput(
				"FieldSimulation/Notes",
				SimulatedArena.getInstance().getGamePiecesByType("Note").toArray(new Pose3d[0]));
	}
}
