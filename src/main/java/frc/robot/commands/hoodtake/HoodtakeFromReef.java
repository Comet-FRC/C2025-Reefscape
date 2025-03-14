package frc.robot.commands.hoodtake;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hoodtake.Hoodtake;

public class HoodtakeFromReef extends WrapperCommand {
    public HoodtakeFromReef(Drive drive, Hoodtake hoodtake) {
        super(
            Commands.defer( // we have to defer here so that this runs at runtime
                () -> Commands.either(
                    new HoodtakeFromL3Auto(drive, hoodtake),
                    new HoodtakeFromL2Auto(drive, hoodtake),
                    () -> drive.getTargetAlgae().id() % 2 == 0
                ),
                Set.of(drive, hoodtake)
            )
        );
    }
}
