package frc.robot.subsystems.vision.algae;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.*;


public class AlgaeVisionConstants {
    public static final Distance VISION_DISTANCE_THRESHOLD = Meters.of(2.5);
    public static final double TIME_THRESHOLD = 7.0;

    public static class LIMELIGHT{
    public static final Angle CAMERA_MOUNT_ANGLE = Degrees.of(320);
    public static final Distance TARGET_HEIGHT = Inches.of(240);
    public static final Distance CAMERA_HEIGHT = Inches.of(59.6);
    public static final double X_THRESHOLD = 0.5; //change all thresholds
    public static final double Y_THRESHOLD = 0.5;
    public static final double AREA_THRESHOLD = 0.5;
    }
 
}
