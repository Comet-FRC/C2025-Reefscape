/*
package frc.robot.subsystems.vision.algae;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.VirtualSubsystem;
import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class AlgaeVision extends VirtualSubsystem {
  private final AlgaeVisionIO io;
  private final AlgaeVisionIOInputsAutoLogged inputs = new AlgaeVisionIOInputsAutoLogged();

  private final ArrayList<TrackedAlgae> algaeMemories = new ArrayList<>();

  private static final LoggedNetworkNumber updateDistanceThreshold =
      new LoggedNetworkNumber("TunableNumbers/Vision/Algae/updateDistanceThreshold", 5);
  private static final LoggedNetworkNumber posUpdatingFilteringFactor =
      new LoggedNetworkNumber("TunableNumbers/Vision/Algae/posUpdatingFilteringFactor", 0.8);
  private static final LoggedNetworkNumber confUpdatingFilteringFactor =
      new LoggedNetworkNumber(
          "TunableNumbers/Vision/Algae/Confidence/UpdatingFilteringFactor", 0.5);
  public static final LoggedNetworkNumber confidencePerAreaPercent =
      new LoggedNetworkNumber("TunableNumbers/Vision/Algae/Confidence/PerAreaPercent", 1);
  private static final LoggedNetworkNumber confidenceDecayPerSecond =
      new LoggedNetworkNumber("TunableNumbers/Vision/Algae/Confidence/DecayPerSecond", 3);
  private static final LoggedNetworkNumber priorityPerConfidence =
      new LoggedNetworkNumber("TunableNumbers/Vision/Algae/Priority/PriorityPerConfidence", 4);
  private static final LoggedNetworkNumber priorityPerDistance =
      new LoggedNetworkNumber("TunableNumbers/Vision/Algae/Priority/PriorityPerDistance", -2);
  private static final LoggedNetworkNumber acquireConfidenceThreshold =
      new LoggedNetworkNumber("TunableNumbers/Vision/Algae/Target Threshold/Acquire", -2);
  private static final LoggedNetworkNumber detargetConfidenceThreshold =
      new LoggedNetworkNumber("TunableNumbers/Vision/Algae/Target Threshold/Detarget", -3);

  private Optional<TrackedAlgae> optIntakeTarget = Optional.empty();
  private boolean intakeTargetLocked = false;

  public AlgaeVision(AlgaeVisionIO io) {
    System.out.println("[Init AlgaeVision] Instantiating AlgaeVision");
    this.io = io;
    System.out.println("[Init AlgaeVision] AlgaeVision IO: " + this.io.getClass().getSimpleName());

    CommandScheduler.getInstance()
        .onCommandFinish(
            (command) -> {
              if (command.getName() == IntakeCommand.INTAKE.name()) {
                algaeMemories.stream()
                    .sorted(
                        (a, b) ->
                            (int)
                                Math.signum(
                                    Drive.getInstance()
                                        .getDistanceFrom(a.fieldPos)
                                        .minus(Drive.getInstance().getDistanceFrom(b.fieldPos))
                                        .in(Meters)))
                    .findFirst()
                    .ifPresent(algaeMemories::remove);
                // optIntakeTarget.ifPresent((target) -> algaeMemories.remove(target));
                optIntakeTarget = Optional.empty();
              }
            });

    // new Trigger(DriverStation::isDisabled).debounce(1).whileTrue(new FillAnimation(2, () ->
    // (inputs.connected ? Color.kGreen : Color.kOrange), connectedStrip));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("AlgaeVision", inputs);
    var frameTargets = Arrays.asList(inputs.trackedAlgae);
    var connections = new ArrayList<PhotonMemoryConnection>();

    for (TrackedAlgae memory : algaeMemories) {}

    algaeMemories.forEach(
        (memory) ->
            frameTargets.forEach(
                (target) -> {
                  if (memory.fieldPos.getDistance(target.fieldPos)
                      < updateDistanceThreshold.get()) {
                    connections.add(new PhotonMemoryConnection(memory, target));
                  }
                }));
    connections.sort((a, b) -> (int) Math.signum(a.getDistance() - b.getDistance()));
    var unusedMemories = new ArrayList<>(algaeMemories);
    var unusedTargets = new ArrayList<>(frameTargets);
    while (!connections.isEmpty()) {
      var confirmedConnection = connections.get(0);
      confirmedConnection.memory.updatePosWithFiltering(confirmedConnection.photonFrameTarget);
      confirmedConnection.memory.updateConfidence();
      unusedMemories.remove(confirmedConnection.memory);
      unusedTargets.remove(confirmedConnection.photonFrameTarget);
      connections.removeIf(
          (connection) ->
              connection.memory == confirmedConnection.memory
                  || connection.photonFrameTarget == confirmedConnection.photonFrameTarget);
    }
    unusedMemories.forEach(
        (memory) -> {
          if (Drive.getInstance().getPose().getTranslation().getDistance(memory.fieldPos)
              > RobotConstants.robotLengthMeters * 0.5) {
            memory.decayConfidence(1);
          }
        });
    unusedTargets.forEach((target) -> algaeMemories.add(target));
    algaeMemories.removeIf((memory) -> memory.confidence <= 0);
    algaeMemories.removeIf((memory) -> Double.isNaN(memory.fieldPos.getX()));
    algaeMemories.removeIf(
        (memory) -> Drive.getInstance().getDistanceFrom(memory.fieldPos).in(Meters) <= 0.07);

    if (optIntakeTarget.isPresent()
        && (optIntakeTarget.get().confidence < detargetConfidenceThreshold.get()
            || !algaeMemories.contains(optIntakeTarget.get()))) {
      optIntakeTarget = Optional.empty();
    }
    if (optIntakeTarget.isEmpty() || !intakeTargetLocked) {
      optIntakeTarget =
          algaeMemories.stream()
              .filter((target) -> target.getPriority() >= acquireConfidenceThreshold.get())
              .sorted((a, b) -> (int) Math.signum(b.getPriority() - a.getPriority()))
              .findFirst();
    }

    // Logger.recordOutput("Vision/Algae/Photon Frame Targets",
    // frameTargets.stream().map(AlgaeVision::targetToPose).toArray(Pose3d[]::new));
    Logger.recordOutput(
        "Vision/Algae/Algae Memories",
        algaeMemories.stream().map(TrackedAlgae::toASPose).toArray(Pose3d[]::new));
    Logger.recordOutput(
        "Vision/Algae/Algae Confidence",
        algaeMemories.stream().mapToDouble((algae) -> algae.confidence).toArray());
    Logger.recordOutput(
        "Vision/Algae/Algae Priority",
        algaeMemories.stream().mapToDouble(TrackedAlgae::getPriority).toArray());
    Logger.recordOutput(
        "Vision/Algae/Target",
        optIntakeTarget
            .map(TrackedAlgae::toASPose)
            .map((a) -> new Pose3d[] {a})
            .orElse(new Pose3d[0]));
    Logger.recordOutput(
        "Vision/Algae/Locked Target",
        optIntakeTarget
            .filter((a) -> intakeTargetLocked)
            .map(TrackedAlgae::toASPose)
            .map((a) -> new Pose3d[] {a})
            .orElse(new Pose3d[0]));
  }

  public DoubleSupplier applyDotProduct(Supplier<ChassisSpeeds> joystickFieldRelative) {
    return () ->
        optIntakeTarget
            .map(
                (target) -> {
                  var robotTrans = Drive.getInstance().getPose().getTranslation();
                  var targetRelRobot = target.fieldPos.minus(robotTrans);
                  var targetRelRobotNormalized = targetRelRobot.div(targetRelRobot.getNorm());
                  var joystickSpeed = joystickFieldRelative.get();
                  var joy =
                      new Translation2d(
                          joystickSpeed.vxMetersPerSecond, joystickSpeed.vyMetersPerSecond);
                  var throttle = targetRelRobotNormalized.toVector().dot(joy.toVector());
                  return throttle;
                })
            .orElse(0.0);
  }

  public Supplier<Translation2d> autoIntakeTargetLocation() {
    return () -> optIntakeTarget.map((target) -> target.fieldPos).get();
  }

  public boolean hasTarget() {
    return optIntakeTarget.isPresent();
  }

  public boolean targetLocked() {
    return intakeTargetLocked;
  }

  public void clearMemory() {
    algaeMemories.clear();
    optIntakeTarget = Optional.empty();
  }

  public Command autoIntake(DoubleSupplier throttle, Drive drive, Intake intake) {
    return Commands.runOnce(() -> intakeTargetLocked = true)
        .alongWith(
            drive.translationSubsystem.fieldRelative(
                getAutoIntakeTransSpeed(throttle).orElseGet(ChassisSpeeds::new)),
            drive.rotationalSubsystem.pointTo(
                autoIntakeTargetLocation(), () -> RobotConstants.intakeForward))
        .onlyWhile(() -> !intake.hasAlgae() && optIntakeTarget.isPresent())
        .finallyDo(() -> intakeTargetLocked = false)
        .withName("Auto Intake");
  }

  private static record PhotonMemoryConnection(
      TrackedAlgae memory, TrackedAlgae photonFrameTarget) {
    public double getDistance() {
      return memory.fieldPos.getDistance(photonFrameTarget.fieldPos);
    }
  }

  public static class TrackedAlgae implements StructSerializable {
    public Translation2d fieldPos;
    public double confidence;

    public TrackedAlgae(Translation2d fieldPos, double confidence) {
      this.fieldPos = fieldPos;
      this.confidence = confidence * confUpdatingFilteringFactor.get();
    }

    public void updateConfidence() {
      confidence += confidence * MathUtil.clamp(1 - confUpdatingFilteringFactor.get(), 0, 1);
    }

    public void updatePosWithFiltering(TrackedAlgae newAlgae) {
      this.fieldPos = fieldPos.interpolate(newAlgae.fieldPos, posUpdatingFilteringFactor.get());
      this.confidence = newAlgae.confidence;
    }

    public void decayConfidence(double rate) {
      this.confidence -= confidenceDecayPerSecond.get() * rate * Constants.dtSeconds;
    }

    public double getPriority() {
      var pose = RobotState.getInstance().getPose();
      var FORR = fieldPos.minus(pose.getTranslation());
      var rotation = pose.getRotation().minus(RobotConstants.intakeForward);
      return confidence
              * priorityPerConfidence.get()
              * VecBuilder.fill(rotation.getCos(), rotation.getSin()).dot(FORR.toVector().unit())
          + FORR.getNorm() * priorityPerDistance.get();
    }

    public Pose3d toASPose() {
      return new Pose3d(
          new Translation3d(fieldPos.getX(), fieldPos.getY(), Units.inchesToMeters(1)),
          new Rotation3d());
    }

    public static final TrackedAlgaeStruct struct = new TrackedAlgaeStruct();

    public static class TrackedAlgaeStruct implements Struct<TrackedAlgae> {
      @Override
      public Class<TrackedAlgae> getTypeClass() {
        return TrackedAlgae.class;
      }

      @Override
      public String getTypeString() {
        return "struct:TrackedAlgae";
      }

      @Override
      public int getSize() {
        return kSizeDouble + Translation2d.struct.getSize();
      }

      @Override
      public String getSchema() {
        return "Translation2d fieldPos;double confidence";
      }

      @Override
      public Struct<?>[] getNested() {
        return new Struct<?>[] {Translation2d.struct};
      }

      @Override
      public TrackedAlgae unpack(ByteBuffer bb) {
        var fieldPos = Translation2d.struct.unpack(bb);
        var confidence = bb.getDouble();
        return new TrackedAlgae(fieldPos, confidence);
      }

      @Override
      public void pack(ByteBuffer bb, TrackedAlgae value) {
        Translation2d.struct.pack(bb, value.fieldPos);
        bb.putDouble(value.confidence);
      }

      @Override
      public String getTypeName() {
        // TODO: Check if this is a correct implementation
        return "TrackedAlgae";
      }
    }
  }
}
*/
