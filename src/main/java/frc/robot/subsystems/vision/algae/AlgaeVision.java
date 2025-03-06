package frc.robot.subsystems.vision.algae;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.algae.AlgaeVisionIO.AlgaeVisionIOInputs;

public class AlgaeVision extends SubsystemBase {
  private final AlgaeVisionIOLimelight limelight;
  private final AlgaeVisionIOInputs inputs = new AlgaeVisionIOInputs(); // Store inputs

  public AlgaeVision(AlgaeVisionIOLimelight limelight) {
    this.limelight = limelight; // Store the passed-in limelight instance
  }

  @Override
  public void periodic() {
    limelight.updateInputs(inputs); // Call updateInputs with stored inputs
  }
}