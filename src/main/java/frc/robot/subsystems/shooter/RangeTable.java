package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.measure.Distance;

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
        RANGE_TABLE.put(0.0, new ShooterSpeed(0, 0));
        RANGE_TABLE.put(10.0, new ShooterSpeed(5000, 5000));
	}

    /**
     * returns the ShooterSpeed value corresponding to a given distance from the speaker
     * 
     * @param distance the distance at which the shooter speed needs to be determined. 
     * @return The speeds to rev the shooter motors at
     */
    public Supplier<ShooterSpeed> get(DoubleSupplier distance) {
        return () -> this.RANGE_TABLE.get(distance.getAsDouble());
    }

    /**
     * returns the ShooterSpeed value corresponding to a given distance from the speaker
     * 
     * @param distance the distance at which the shooter speed needs to be determined. 
     * @return The speeds to rev the shooter motors at
     */
    public Supplier<ShooterSpeed> get(Supplier<Distance> distance) {
        return this.get(() -> distance.get().in(Meters));
    }
}