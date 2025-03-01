package frc.robot.subsystems.vision.algae;


import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.vision.algae.AlgaeVision.TrackedAlgae;

public interface AlgaeVisionIO {
  @AutoLog
  public static class AlgaeVisionIOInputs {
    public boolean connected;
    public TrackedAlgae[] trackedAlgae = new TrackedAlgae[0];
  }
  
  public default void updateInputs(AlgaeVisionIOInputs inputs) {}
}

