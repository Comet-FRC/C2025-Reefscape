package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hoodtake.Hoodtake;

public class IntakeFromReef extends DeferredCommand {
    public IntakeFromReef(Drive drive, Hoodtake hoodtake) {
        super(
            () -> Commands.either(
                new IntakeFromL3(drive, hoodtake),
                new IntakeFromL2(drive, hoodtake),
                () -> drive.getTargetAlgae().id() % 2 == 0
            ),
            Set.of(drive, hoodtake)
        );
    }
}
