package frc.team2412.robot.util;

import java.io.BufferedReader;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;

public class LauncherDataLoader {

    public static InterpolatingTreeMap<Double, LauncherDataPoint> fromCSV(Path path) {
        InterpolatingTreeMap<Double, LauncherDataPoint> map = new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), LauncherDataPoint.INTERPOLATOR);

        try {
            BufferedReader reader = Files.newBufferedReader(path);
        } catch (IOException e) {
            e.printStackTrace();
        }
        

        // TODO: make this actually use the csv file

        map.put(1.0, new LauncherDataPoint(0, 0));
        map.put(10.0, new LauncherDataPoint(1.0, 2.0));

        return map;
    }

}
