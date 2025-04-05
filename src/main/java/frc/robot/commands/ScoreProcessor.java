package frc.robot.commands;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;

public class ScoreProcessor extends SequentialCommandGroup {
    public ScoreProcessor(Intake intake, Indexer indexer) {
        super(
            intake.setWheelVoltage(() -> Volts.of(-2)),
            intake.setPivotPosition(() -> Degrees.of(90)),
            indexer.setRightVoltage(() -> Volts.of(2)),
            Commands.waitUntil(() -> indexer.getRightPosition().gt(Degrees.of(150))),
            Commands.waitSeconds(0.2),
            indexer.setRightVoltage(() -> Volts.of(0))
        );
    }
}
