package frc.robot.subsystems.vision.algae;

import java.util.ArrayList;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public interface AlgaeVisionIO {
  @AutoLog
  public static class AlgaeVisionIOInputs {
    public boolean connected;
    public ArrayList<TrackedAlgae> AlgaePoses = new ArrayList<>();

  }

  public static class TrackedAlgae {
    public static double timestamp;
    public Pose2d pose;
    public double confidence;
    public TrackedAlgae(double timestamp, Pose2d pose, double confidence) {
      this.timestamp = timestamp;
      this.pose = pose;
      this.confidence = confidence;
    }
    public Translation2d getTranslation() {
      return pose.getTranslation();
    }
    public Pose2d getPose() {
      return pose;
    }
    public double getConfidence() {
      return confidence;
    } 
    public double getTimestamp() {
      return timestamp;
    }
  }
  
  public default void updateInputs(AlgaeVisionIOInputs inputs) {}
}

