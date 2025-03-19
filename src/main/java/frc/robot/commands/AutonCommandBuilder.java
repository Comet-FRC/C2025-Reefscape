package frc.robot.commands;

import org.littletonrobotics.junction.networktables.LoggedNetworkInput;
import org.littletonrobotics.junction.networktables.LoggedNetworkString;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutonCommandBuilder {
    private static LoggedNetworkString strategyInput = 
        new LoggedNetworkString("/SmartDashboard/Auton", "DEC");

    public Command getCommand() {
        Command command = Commands.none();
        String input = strategyInput.get();

        for (int i=0; i<input.length(); ++i) {
            
        }


        return command;
    }
}
