package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import org.ironmaple.utils.FieldMirroringUtils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.util.AllianceColor;

public class AutoScoreProcessor extends WrapperCommand {
    public AutoScoreProcessor(Drive drive, Intake intake, Indexer indexer) {
        super(
            Commands.sequence(
                drive.moveToPosePID(() -> {
                    Pose2d pose = new Pose2d(new Translation2d(Meters.of(6.13), Meters.of(0.5)), Rotation2d.fromDegrees(180));
                    return FieldMirroringUtils.toCurrentAlliancePose(pose);
                }),
				intake.setWheelVoltage(() -> Volts.of(-3)),
				intake.setPivotPosition(() -> Degrees.of(85)),
				indexer.setRightVoltage(() -> Volts.of(3))
			)
        );
    }
}
