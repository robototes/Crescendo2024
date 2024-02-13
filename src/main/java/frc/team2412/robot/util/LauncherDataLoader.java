package frc.team2412.robot.util;

import java.io.File;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;

public class LauncherDataLoader {

    public static InterpolatingTreeMap<Double, LauncherDataPoint> fromCSV(File file) {
        InterpolatingTreeMap<Double, LauncherDataPoint> map = new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), LauncherDataPoint.INTERPOLATOR);

        // TODO: make this actually use the csv file

        map.put(1.0, new LauncherDataPoint(0, 0));
        map.put(10.0, new LauncherDataPoint(1.0, 2.0));

        return map;
    }

}
