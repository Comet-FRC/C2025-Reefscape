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
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.branch_selector.BranchSelector;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOMapleSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.drive.gyro.GyroIO;
import frc.robot.subsystems.drive.gyro.GyroIOPigeon2;
import frc.robot.subsystems.drive.gyro.GyroIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.controller.CometController;
import frc.robot.util.controller.CometPS4Controller;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final DriveSubsystem drive;

  private final Vision vision;

  // Controller
  private final CometController controller = new CometPS4Controller(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  private SwerveDriveSimulation swerveDriveSimulation;

  // values
  private BranchSelector branchSelector = new BranchSelector();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new DriveSubsystem(
                new GyroIOPigeon2(),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3));
        this.vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOLimelight("limelight-shooter", drive::getRotation));
        break;

      case SIM:
        DriveTrainSimulationConfig driveTrainSimulationConfig =
            DriveTrainSimulationConfig.Default()
                .withGyro(COTS.ofPigeon2())
                .withSwerveModule(
                    COTS.ofMark4i(DCMotor.getNEO(1), DCMotor.getNEO(1), COTS.WHEELS.COLSONS.cof, 1))
                .withTrackLengthTrackWidth(Meters.of(0.3429), Meters.of(0.4953))
                .withBumperSize(Meters.of(0.84), Meters.of(0.87));

        this.swerveDriveSimulation =
            new SwerveDriveSimulation(
                // Specify Configuration
                driveTrainSimulationConfig,
                // Specify starting pose
                new Pose2d(10, 5, new Rotation2d()));

        // Register the drivetrain simulation to the default simulation world
        SimulatedArena.getInstance().addDriveTrainSimulation(swerveDriveSimulation);

        // drive = new DriveSubsystem(
        // new GyroIO() {
        // },
        // new ModuleIOSim(),
        // new ModuleIOSim(),
        // new ModuleIOSim(),
        // new ModuleIOSim());

        this.drive =
            new DriveSubsystem(
                new GyroIOSim(this.swerveDriveSimulation.getGyroSimulation()),
                new ModuleIOMapleSim(this.swerveDriveSimulation.getModules()[0]),
                new ModuleIOMapleSim(this.swerveDriveSimulation.getModules()[1]),
                new ModuleIOMapleSim(this.swerveDriveSimulation.getModules()[2]),
                new ModuleIOMapleSim(this.swerveDriveSimulation.getModules()[3]));

        this.vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(
                    "limelight-shooter",
                    robotToCamera0,
                    swerveDriveSimulation::getSimulatedDriveTrainPose));
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new DriveSubsystem(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        this.vision = new Vision(drive::addVisionMeasurement, new VisionIO() {});
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();

    Logger.recordOutput("Driver/selectedBranch", 'A');
    Logger.recordOutput("Driver/selectedBranchLevel", 4);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> controller.getRightX()));

    controller.left().whileTrue(Commands.runOnce(() -> {
        Logger.recordOutput("Driver/selectedBranch", branchSelector.prevBranch());
    }));
    controller.right().whileTrue(Commands.runOnce(() -> {
        Logger.recordOutput("Driver/selectedBranch", branchSelector.nextBranch());
    }));
    controller.down().whileTrue(Commands.runOnce(() -> {
        Logger.recordOutput("Driver/selectedBranchLevel", branchSelector.prevLevel());
    }));
    controller.up().whileTrue(Commands.runOnce(() -> {
        Logger.recordOutput("Driver/selectedBranchLevel", branchSelector.nextLevel());
    }));
    controller.b().whileTrue(DriveCommands.feedforwardCharacterization(drive));

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
                    drive)
                .ignoringDisable(true));
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
    if (Constants.currentMode != Constants.Mode.SIM) return;

    Logger.recordOutput(
        "FieldSimulation/RobotPosition", swerveDriveSimulation.getSimulatedDriveTrainPose());
    Logger.recordOutput(
        "FieldSimulation/Notes",
        SimulatedArena.getInstance().getGamePiecesByType("Note").toArray(new Pose3d[0]));
  }
}
