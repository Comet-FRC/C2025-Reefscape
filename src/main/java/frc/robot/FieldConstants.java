// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import frc.robot.subsystems.drive.TargetAlgae;
import frc.robot.util.AllianceColor;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.littletonrobotics.junction.Logger;

//Generously Borrowed from FRC-6328

/**
 * Contains various field dimensions and useful reference points. All units are
 * in meters and poses
 * have a blue alliance origin.
 */
public class FieldConstants {
	public static final double fieldLength = Units.inchesToMeters(690.876);
	public static final double fieldWidth = Units.inchesToMeters(317);
	public static final double startingLineX = Units.inchesToMeters(299.438); // Measured from the inside of starting
																				// line
	public static final double algaeDiameter = Units.inchesToMeters(16);

	public static class Processor {
		public static final Pose2d centerFace = new Pose2d(Units.inchesToMeters(235.726), 0,
				Rotation2d.fromDegrees(90));
	}

	public static class Barge {
		public static final Distance BARGE_X = Inches.of(345.428);

		public static final Translation2d startOfRedBarge = new Translation2d(Inches.of(345.428),
				Inches.of(10));
		public static final Translation2d endOfRedBarge = new Translation2d(Inches.of(345.428),
				Inches.of(146.50));
		public static final Translation2d endOfBlueBarge = new Translation2d(Inches.of(345.428),
				Inches.of(156.50));
		public static final Translation2d startOfBlueBarge = new Translation2d(Inches.of(345.428),
				Inches.of(293));

		// Measured from floor to bottom of cage
		public static final double deepHeight = Units.inchesToMeters(3.125);
		public static final double shallowHeight = Units.inchesToMeters(30.125);
	}

	public static class CoralStation {
		public static final Pose2d leftCenterFace = new Pose2d(
				Units.inchesToMeters(33.526),
				Units.inchesToMeters(291.176),
				Rotation2d.fromDegrees(90 - 144.011));
		public static final Pose2d rightCenterFace = new Pose2d(
				Units.inchesToMeters(33.526),
				Units.inchesToMeters(25.824),
				Rotation2d.fromDegrees(144.011 - 90));
	}

	public static class Reef {
		public static final Translation2d center = new Translation2d(Units.inchesToMeters(176.746),
				Units.inchesToMeters(158.501));
		public static final double faceToZoneLine = Units.inchesToMeters(12); // Side of the reef to the inside of the
																				// reef zone line

		public static final Pose2d[] centerFaces = new Pose2d[6]; // Starting facing the driver station in clockwise
																	// order
		public static final Pose2d[] reefAlgaeTargetPoses = new Pose2d[6];
		public static final Pose2d[] reefAlgaeTargetPosesOpposingSide = new Pose2d[6];

		public static final List<Map<ReefHeight, Pose3d>> branchPositions = new ArrayList<>(); // Starting at the right
																								// branch facing the
																								// driver station in
																								// clockwise

		static {
			// Initialize faces
			centerFaces[0] = new Pose2d(Inches.of(144.003), Inches.of(158.500), Rotation2d.fromDegrees(180));
			centerFaces[1] = new Pose2d(Inches.of(160.373), Inches.of(186.857), Rotation2d.fromDegrees(120));
			centerFaces[2] = new Pose2d(Inches.of(193.116), Inches.of(186.858), Rotation2d.fromDegrees(60));
			centerFaces[3] = new Pose2d(Inches.of(209.489), Inches.of(158.502), Rotation2d.fromDegrees(0));
			centerFaces[4] = new Pose2d(Inches.of(193.118), Inches.of(130.145), Rotation2d.fromDegrees(-60));
			centerFaces[5] = new Pose2d(Inches.of(160.375), Inches.of(130.144), Rotation2d.fromDegrees(-120));

			for (int i = 0; i < 6; ++i) {

				Translation2d relativeTranslation = centerFaces[i].getTranslation().minus(center).times(1.5);

				System.out.println("Relative Translation: " + relativeTranslation);

				reefAlgaeTargetPoses[i] = new Pose2d(
						centerFaces[i].getX() + relativeTranslation.getX(),
						centerFaces[i].getY() + relativeTranslation.getY(),
						centerFaces[i].getRotation());

				reefAlgaeTargetPosesOpposingSide[i] = new Pose2d(
						FieldConstants.fieldLength - reefAlgaeTargetPoses[i].getX(),
						reefAlgaeTargetPoses[i].getY(),
						reefAlgaeTargetPoses[i].getRotation().times(-1).plus(Rotation2d.fromDegrees(180)));
			}

			// Initialize branch positions
			for (int face = 0; face < 6; face++) {
				Map<ReefHeight, Pose3d> fillRight = new HashMap<>();
				Map<ReefHeight, Pose3d> fillLeft = new HashMap<>();
				for (var level : ReefHeight.values()) {
					Pose2d poseDirection = new Pose2d(center, Rotation2d.fromDegrees(180 - (60 * face)));
					double adjustX = Units.inchesToMeters(30.738);
					double adjustY = Units.inchesToMeters(6.469);

					fillRight.put(
							level,
							new Pose3d(
									new Translation3d(
											poseDirection
													.transformBy(new Transform2d(adjustX, adjustY, new Rotation2d()))
													.getX(),
											poseDirection
													.transformBy(new Transform2d(adjustX, adjustY, new Rotation2d()))
													.getY(),
											level.height),
									new Rotation3d(
											0,
											Units.degreesToRadians(level.pitch),
											poseDirection.getRotation().getRadians())));
					fillLeft.put(
							level,
							new Pose3d(
									new Translation3d(
											poseDirection
													.transformBy(new Transform2d(adjustX, -adjustY, new Rotation2d()))
													.getX(),
											poseDirection
													.transformBy(new Transform2d(adjustX, -adjustY, new Rotation2d()))
													.getY(),
											level.height),
									new Rotation3d(
											0,
											Units.degreesToRadians(level.pitch),
											poseDirection.getRotation().getRadians())));
				}
				branchPositions.add(fillRight);
				branchPositions.add(fillLeft);
			}
		}

		public static Pose2d[] getTeamAlgaePoses(boolean isOnOpposingAllianceField) {
			return isOnOpposingAllianceField ? reefAlgaeTargetPosesOpposingSide : reefAlgaeTargetPoses;
		}

		/**
		 * Returns a Pose2d that is `d` meters away from the center along the direction
		 * of a given face.
		 * 
		 * @param target The target algae
		 * @param d     The distance from the center.
		 * @return The computed Pose2d.
		 */
		public static Pose2d getTranslatedPose(TargetAlgae target, Distance d) {
			int index = target.id();
			boolean isOpposingReef = target.isOpposingReef();

			if (index < 0 || index >= centerFaces.length) {
				throw new IllegalArgumentException("Index out of bounds: " + index);
			}

			Pose2d face = centerFaces[index];

			Translation2d relativeTranslation = face.getTranslation().minus(center);

			// normalize vector, then set to the length we want
			relativeTranslation = relativeTranslation.div(relativeTranslation.getNorm());
			relativeTranslation = relativeTranslation.times(d.in(Meters));

			Pose2d translatedPose = new Pose2d(
					face.getX() + relativeTranslation.getX(),
					face.getY() + relativeTranslation.getY(),
					face.getRotation());

			if (isOpposingReef) {
				translatedPose = new Pose2d(
						FieldConstants.fieldLength - translatedPose.getX(),
						translatedPose.getY(),
						translatedPose.getRotation().times(-1).plus(Rotation2d.fromDegrees(180)));
			}

			Logger.recordOutput("Automation/translatedPose", translatedPose);
			return translatedPose;
		}

	}

	public static class StagingPositions {
		// Measured from the center of the ice cream
		public static final Pose2d leftIceCream = new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(230.5),
				new Rotation2d());
		public static final Pose2d middleIceCream = new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(158.5),
				new Rotation2d());
		public static final Pose2d rightIceCream = new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(86.5),
				new Rotation2d());
	}

	public enum ReefHeight {
		L4(Units.inchesToMeters(72), -90),
		L3(Units.inchesToMeters(47.625), -35),
		L2(Units.inchesToMeters(31.875), -35),
		L1(Units.inchesToMeters(18), 0);

		ReefHeight(double height, double pitch) {
			this.height = height;
			this.pitch = pitch; // in degrees
		}

		public final double height;
		public final double pitch;
	}

	/*
	 * public static final double aprilTagWidth = Units.inchesToMeters(6.50);
	 * public static final int aprilTagCount = 22;
	 * public static final AprilTagLayoutType defaultAprilTagType =
	 * AprilTagLayoutType.NO_BARGE;
	 * 
	 * @Getter
	 * public enum AprilTagLayoutType {
	 * OFFICIAL("2025-official"),
	 * NO_BARGE("2025-no-barge"),
	 * BLUE_REEF("2025-blue-reef"),
	 * RED_REEF("2025-red-reef");
	 * 
	 * AprilTagLayoutType(String name) {
	 * if (Constants.disableHAL) {
	 * layout = null;
	 * } else {
	 * try {
	 * layout =
	 * new AprilTagFieldLayout(
	 * Path.of(Filesystem.getDeployDirectory().getPath(), "apriltags", name +
	 * ".json"));
	 * } catch (IOException e) {
	 * throw new RuntimeException(e);
	 * }
	 * }
	 * if (layout == null) {
	 * layoutString = "";
	 * } else {
	 * try {
	 * layoutString = new ObjectMapper().writeValueAsString(layout);
	 * } catch (JsonProcessingException e) {
	 * throw new RuntimeException(
	 * "Failed to serialize AprilTag layout JSON " + toString() + "for Northstar");
	 * }
	 * }
	 * }
	 * 
	 * private final AprilTagFieldLayout layout;
	 * private final String layoutString;
	 * }
	 */
}