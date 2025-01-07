package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs;

  private final PIDController topPID;
  private final PIDController botPID;
  private final SimpleMotorFeedforward ff;

  public Arm(ArmIO io) {
    this.io = io;
    this.inputs = new ArmIOInputsAutoLogged();

    this.topPID =
        new PIDController(
            ArmConstants.gains.kP(), ArmConstants.gains.kI(), ArmConstants.gains.kD());
    this.botPID =
        new PIDController(
            ArmConstants.gains.kP(), ArmConstants.gains.kI(), ArmConstants.gains.kD());

    this.ff =
        new SimpleMotorFeedforward(
            ArmConstants.gains.kS(), ArmConstants.gains.kV(), ArmConstants.gains.kA());
  }
}
