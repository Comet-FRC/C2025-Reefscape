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

package frc.robot.subsystems.vision.algae;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import java.util.function.Supplier;

/** IO implementation for real Limelight hardware. */
public class AlgaeVisionIOLimelight implements AlgaeVisionIO {
  private final Supplier<Rotation2d> rotationSupplier;
  private final DoubleArrayPublisher orientationPublisher;

  private final DoubleSubscriber latencySubscriber;
  private final DoubleSubscriber txSubscriber;
  private final DoubleSubscriber tySubscriber;
  private final DoubleArraySubscriber megatag1Subscriber;
  private final DoubleArraySubscriber megatag2Subscriber;
  public static final Pose2d[] centerFaces =
        new Pose2d[6];

  /**
   * Creates a new VisionIOLimelight.
   *
   * @param name The configured name of the Limelight.
   * @param rotationSupplier Supplier for the current estimated rotation, used for MegaTag 2.
   */
  public AlgaeVisionIOLimelight(String name, Supplier<Rotation2d> rotationSupplier) {
    var table = NetworkTableInstance.getDefault().getTable(name);
    this.rotationSupplier = rotationSupplier;
    orientationPublisher = table.getDoubleArrayTopic("robot_orientation_set").publish();
    latencySubscriber = table.getDoubleTopic("tl").subscribe(0.0);
    txSubscriber = table.getDoubleTopic("tx").subscribe(0.0);
    tySubscriber = table.getDoubleTopic("ty").subscribe(0.0);
    megatag1Subscriber = table.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[] {});
    megatag2Subscriber =
        table.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[] {});
  centerFaces[0] =
    new Pose2d(
        Units.inchesToMeters(144.003),
        Units.inchesToMeters(158.500),
        Rotation2d.fromDegrees(180));
  centerFaces[1] =
    new Pose2d(
        Units.inchesToMeters(160.373),
        Units.inchesToMeters(186.857),
        Rotation2d.fromDegrees(120));
  centerFaces[2] =
    new Pose2d(
        Units.inchesToMeters(193.116),
        Units.inchesToMeters(186.858),
        Rotation2d.fromDegrees(60));
  centerFaces[3] =
    new Pose2d(
        Units.inchesToMeters(209.489),
        Units.inchesToMeters(158.502),
        Rotation2d.fromDegrees(0));
  centerFaces[4] =
    new Pose2d(
        Units.inchesToMeters(193.118),
        Units.inchesToMeters(130.145),
        Rotation2d.fromDegrees(-60));
    centerFaces[5] =
    new Pose2d(
        Units.inchesToMeters(160.375), 
        Units.inchesToMeters(130.144),
        Rotation2d.fromDegrees(-120));
  }

  @Override
  public void updateInputs(AlgaeVisionIOInputs inputs) {
    // Update connection status based on whether an update has been seen in the last 250ms
    inputs.connected = (RobotController.getFPGATime() - latencySubscriber.getLastChange()) < 250;

    
}
}