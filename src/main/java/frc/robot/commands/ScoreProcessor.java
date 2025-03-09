package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;

public class ScoreProcessor extends WrapperCommand {
    public ScoreProcessor(Intake intake, Indexer indexer) {
        super(
            Commands.sequence(
                intake.setPivotPosition(() -> Degrees.of(90.0)),
                Commands.waitUntil(intake::atPosition),
                intake.setWheelVoltage(() -> Volts.of(-2)),
                indexer.setRightVoltage(() -> Volts.of(3))
            )
        );
    }
}
