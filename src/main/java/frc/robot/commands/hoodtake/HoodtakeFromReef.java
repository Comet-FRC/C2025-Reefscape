package frc.robot.commands.hoodtake;

import java.util.Set;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.TargetAlgae;
import frc.robot.subsystems.hoodtake.Hoodtake;

public class HoodtakeFromReef extends WrapperCommand {
    public HoodtakeFromReef(Drive drive, Hoodtake hoodtake, Supplier<TargetAlgae> targetAlgae) {
        super(
            Commands.either(
                new HoodtakeFromL3Auto(drive, hoodtake, targetAlgae),
                new HoodtakeFromL2Auto(drive, hoodtake, targetAlgae),
                () -> targetAlgae.get().id() % 2 == 0
            ).andThen(
                hoodtake.defaultCommand()
            )
        );
    }
}
