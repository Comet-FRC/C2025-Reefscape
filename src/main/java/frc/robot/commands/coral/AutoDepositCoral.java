package frc.robot.commands.coral;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import org.ironmaple.utils.FieldMirroringUtils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;

public class AutoDepositCoral extends SequentialCommandGroup {

    public AutoDepositCoral(Drive drive, Intake intake) {

        super(
            drive.moveToPosePID(() -> {
                    Pose2d pose = new Pose2d(new Translation2d(6.144, 3.580), Rotation2d.fromDegrees(90));
                    return FieldMirroringUtils.toCurrentAlliancePose(pose);
                }
            ),
            intake.setPivotPosition(() -> Degrees.of(50)),
            Commands.waitSeconds(1),
            intake.setWheelVoltage(() -> Volts.of(-2.5)),
            Commands.waitSeconds(1)

        );
    }
}
