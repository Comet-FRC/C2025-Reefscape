package frc.robot.commands.coral;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;

public class DepositCoral extends SequentialCommandGroup {

    public DepositCoral(Drive drive, Intake intake) {
        addRequirements(drive, intake);

        // super(
        //     //drive.joystickDrive(, null, null)
        // );
    }
}
