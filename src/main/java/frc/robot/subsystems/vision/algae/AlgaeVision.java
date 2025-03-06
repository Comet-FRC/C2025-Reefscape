package frc.robot.subsystems.vision.algae;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.algae.AlgaeVisionIO.AlgaeVisionIOInputs;

public class AlgaeVision extends SubsystemBase {
  public final AlgaeVisionIO io;
  private final AlgaeVisionIOInputs inputs; // Store inputs

  public AlgaeVision(AlgaeVisionIO io) {
    this.io = io; // Store the passed-in io instance
    this.inputs = new AlgaeVisionIOInputs(); // Initialize inputs
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs); // Call updateInputs with stored inputs
  }
}