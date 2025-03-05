package frc.robot.subsystems.vision.algae;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TransferQueue;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LimelightHelpers;
import edu.wpi.first.wpilibj.Timer;

public class AlgaeVisionIOLimelight implements AlgaeVisionIO {
  public double tx;
  public String name;
  public double ty;
  public double ta;
  public boolean hasTarget;
  public double area;
  public double confidence;
  public double timestamp;

  public AlgaeVisionIOLimelight(String name) {
    this.name = name;
  }

  public void updateInputs(AlgaeVisionIOInputs inputs) {
    inputs.connected = true;
    tx = LimelightHelpers.getTX(name);
    ty = LimelightHelpers.getTY(name);
    ta = LimelightHelpers.getTA(name);
    hasTarget = LimelightHelpers.getTV(name);
    var AlgaePose = getAlgaePose();

    for (TrackedAlgae existingPoses : inputs.AlgaePoses) {
      double distance = existingPoses.getTranslation().getDistance(AlgaePose.getTranslation());
      if (distance < AlgaeVisionConstants.VISION_DISTANCE_THRESHOLD.in(Meters)) {
        confidence(); //placeholder
      } else {
        timestamp = Timer.getFPGATimestamp();
        inputs.AlgaePoses.add(new TrackedAlgae(timestamp, AlgaePose, 1));
      }
    }

  }

  public Pose2d getAlgaePose() {
    if (hasTarget) {
      double tx = LimelightHelpers.getTX(name);
      double ty = LimelightHelpers.getTY(name);

      // Distance estimation using vertical angle
      double targetAngle = AlgaeVisionConstants.LIMELIGHT.CAMERA_MOUNT_ANGLE.in(Degree) + Units.degreesToRadians(ty);
      double distance = (AlgaeVisionConstants.LIMELIGHT.TARGET_HEIGHT.in(Inches)
          - AlgaeVisionConstants.LIMELIGHT.CAMERA_HEIGHT.in(Inches)) / Math.tan(targetAngle);

      // Get robot pose
      Pose2d robotPose = Drive.getInstance().getPose();
      double robotX = robotPose.getX();
      double robotY = robotPose.getY();
      double robotTheta = robotPose.getRotation().getRadians();

      // Compute field position of the algae
      double objectX = robotX + distance * Math.cos(robotTheta + Units.degreesToRadians(tx));
      double objectY = robotY + distance * Math.sin(robotTheta + Units.degreesToRadians(tx));

      return new Pose2d(objectX, objectY, new Rotation2d());
    } else {
      return new Pose2d(); //TODO: Fix this line or else stack will become too big
    }
  }

//Run for every single Algae in list
public double confidence(TrackedAlgae TrackedAlgae1){
    double timestamp1 = TrackedAlgae1.timestamp;
    double rightNow = Timer.getFPGATimestamp();
    if ( /*IF SEEN ALGAE*/ ) //Implement Vision bounding box of limelight, seeing if the poses fit in the box.
    {
        return 1.0;
    }
    else{
        return -0.01;
    }
}
}
