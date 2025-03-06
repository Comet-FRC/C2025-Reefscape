package frc.robot.subsystems.vision.algae;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeVision extends SubsystemBase {

  public AlgaeVision(AlgaeVisionIOLimelight limelight) {
    limelight = new AlgaeVisionIOLimelight("limelight");
  }

}