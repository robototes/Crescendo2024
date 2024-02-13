package frc.team2412.robot.util;

import edu.wpi.first.math.interpolation.Interpolator;

public class LauncherDataPoint {

    public static final Interpolator<LauncherDataPoint> INTERPOLATOR = new Interpolator<LauncherDataPoint>() {
        @Override
        public LauncherDataPoint interpolate(LauncherDataPoint startValue, LauncherDataPoint endValue, double t) {
            // TODO Auto-generated method stub
            throw new UnsupportedOperationException("Unimplemented method 'interpolate'");
        }
    };

    public final double angle;
    public final double rpm;

    public LauncherDataPoint(double angle, double rpm) {
        this.angle = angle;
        this.rpm = rpm;
    }
    
}
