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
import frc.robot.subsystems.intakeLeft.LeftIntake;

public class AutoDepositCoralE extends SequentialCommandGroup {

    public AutoDepositCoralE(Drive drive, LeftIntake intake) {

        super(
            drive.moveToPosePID(() -> {
                    Pose2d pose = new Pose2d(new Translation2d(5.56658654221, 5.30190688698), Rotation2d.fromDegrees(120));
                    return FieldMirroringUtils.toCurrentAlliancePose(pose);
                }
            ),
            intake.setPivotPosition(() -> Degrees.of(60)),
            Commands.waitSeconds(2),
            intake.setWheelVoltage(() -> Volts.of(-7)),
            Commands.waitSeconds(5)

        );
    }
}
