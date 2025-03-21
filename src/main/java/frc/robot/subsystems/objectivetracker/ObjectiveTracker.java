package frc.robot.subsystems.objectivetracker;

import java.lang.annotation.Target;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkString;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.FieldConstants;
import frc.robot.util.VirtualSubsystem;

public class ObjectiveTracker extends VirtualSubsystem {
	private final LoggedNetworkString reefAlgaeStrategyInput = new LoggedNetworkString("/SmartDashboard/Algae Strategy", "DEC");

	private String getReefAlgaeStrategy() {
		return reefAlgaeStrategyInput.get();
	}

	@Override
	public void periodic() {
		Logger.recordOutput("ObjectiveTracker/Strategy/StrategyInput", this.getReefAlgaeStrategy());
	}
}
