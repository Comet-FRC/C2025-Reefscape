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
            intake.setWheelVoltage(() -> Volts.of(-3)),
            intake.setPivotPosition(() -> Degrees.of(85)),
            indexer.setRightVoltage(() -> Volts.of(3)),
            Commands.waitUntil(() -> indexer.getRightSupplyCurrent().gt(Amps.of(80)))
        );
    }
}
