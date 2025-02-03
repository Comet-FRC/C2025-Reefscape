package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;

public class RangeTable {

    private final InterpolatingTreeMap<Double, ShooterSpeed> RANGE_TABLE;
     
    public RangeTable() {
        this.RANGE_TABLE = new InterpolatingTreeMap<Double, ShooterSpeed>(
            InverseInterpolator.forDouble(),
			ShooterSpeed.getInterpolator()
        );

        this.setupRangeTable();
    }

    private void setupRangeTable() {
		RANGE_TABLE.put(1.14, new ShooterSpeed(RPM.of(6000), RPM.of(4500)));
		RANGE_TABLE.put(1.33, new ShooterSpeed(RPM.of(3600), RPM.of(3400)));
		RANGE_TABLE.put(1.48, new ShooterSpeed(RPM.of(3500), RPM.of(3500)));
		RANGE_TABLE.put(1.87, new ShooterSpeed(RPM.of(3000), RPM.of(2800)));
		RANGE_TABLE.put(2.12, new ShooterSpeed(RPM.of(2800), RPM.of(2800)));
		RANGE_TABLE.put(2.36, new ShooterSpeed(RPM.of(2700), RPM.of(2600)));
		RANGE_TABLE.put(2.53, new ShooterSpeed(RPM.of(2700), RPM.of(2350)));
	}

    /**
     * returns the ShooterSpeed value corresponding to a given distance from the speaker
     * 
     * @param distance the distance at which the shooter speed needs to be determined. 
     * @return The speeds to rev the shooter motors at
     */
    public ShooterSpeed get(double distance) {
        return this.RANGE_TABLE.get(distance);
    }
}