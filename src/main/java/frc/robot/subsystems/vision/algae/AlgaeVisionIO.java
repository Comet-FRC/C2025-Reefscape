package frc.robot.subsystems.vision.algae;


import org.littletonrobotics.junction.AutoLog;

public interface AlgaeVisionIO {
  @AutoLog
  public static class AlgaeVisionIOInputs {
    public boolean connected;
  }
  
  public default void updateInputs(AlgaeVisionIOInputs inputs) {}
}

