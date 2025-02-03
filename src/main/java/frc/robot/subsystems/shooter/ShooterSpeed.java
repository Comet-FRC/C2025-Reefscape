package frc.robot.subsystems.shooter;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.units.measure.AngularVelocity;

/**
 * Represents the speeds of the top and bottom motors of the shooter.
 */
public class ShooterSpeed implements Interpolatable<ShooterSpeed> {
    AngularVelocity topMotorSpeed;
    AngularVelocity botMotorSpeed;

    public ShooterSpeed() {
        this.topMotorSpeed = RadiansPerSecond.of(0);
        this.botMotorSpeed = RadiansPerSecond.of(0);
    }

    public ShooterSpeed(AngularVelocity topMotorSpeed, AngularVelocity bottomMotorSpeed) {
        this.topMotorSpeed = topMotorSpeed;
        this.botMotorSpeed = bottomMotorSpeed;
    }

    private ShooterSpeed diff (ShooterSpeed o) {
        return new ShooterSpeed(
            this.topMotorSpeed.minus(o.topMotorSpeed),
            this.botMotorSpeed.minus(o.botMotorSpeed)
        );
    }

    private ShooterSpeed sum(ShooterSpeed o) {
        return new ShooterSpeed(
            this.topMotorSpeed.plus(o.topMotorSpeed),
            this.botMotorSpeed.plus(o.botMotorSpeed)
        );
    }

    private ShooterSpeed product(double scalar) {
        return new ShooterSpeed(
            this.topMotorSpeed.times(scalar),
            this.botMotorSpeed.times(scalar)
        );
    }

    @Override
    public ShooterSpeed interpolate(ShooterSpeed endValue, double t) {
        ShooterSpeed delta = this.diff(endValue);
        ShooterSpeed interpolated = this.sum(delta.product(t));
        return interpolated;
    }

    public AngularVelocity getBotMotorSpeed() {
        return botMotorSpeed;
    }

    public AngularVelocity getTopMotorSpeed(){
        return topMotorSpeed;
    }

    /**
     * Returns interpolator for Double.
     *
     * @return Interpolator for Double.
     */
    static Interpolator<ShooterSpeed> getInterpolator() {
        return ShooterSpeed::interpolate;
    }
}