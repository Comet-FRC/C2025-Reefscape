package frc.robot.commands.hoodtake;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hoodtake.Hoodtake;

public class HoodtakeFromReef extends DeferredCommand {
    public HoodtakeFromReef(Drive drive, Hoodtake hoodtake) {
        super(
            () -> Commands.either(
                new HoodtakeFromL3(drive, hoodtake),
                new HoodtakeFromL2(drive, hoodtake),
                () -> drive.getTargetAlgae().id() % 2 == 0
            ),
            Set.of(drive, hoodtake)
        );
    }
}
