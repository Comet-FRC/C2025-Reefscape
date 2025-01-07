package frc.robot.subsystems.arm;

import frc.robot.Constants;

public class ArmConstants {
  public static final Gains gains =
      switch (Constants.currentMode) {
        case SIM -> new Gains(0.05, 0.0, 0.0, 0.01, 0.00103, 0.0);
        default -> new Gains(0.18, 0, 0.0006, 0.38367, 0.00108, 0);
      };

  public record Gains(double kP, double kI, double kD, double kS, double kV, double kA) {}
}
