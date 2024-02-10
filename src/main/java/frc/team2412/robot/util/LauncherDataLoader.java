package frc.team2412.robot.util;

import java.io.BufferedReader;
import java.io.File;
import java.nio.file.Files;
import java.nio.file.Paths;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.wpilibj.Filesystem;

public class LauncherDataLoader {

    private static final String CSV_DATA_PATH = "launcher_data.csv";

    public static InterpolatingTreeMap<Double, LauncherDataPoint> fromCSV() {
        InterpolatingTreeMap<Double, LauncherDataPoint> map = new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), LauncherDataPoint.INTERPOLATOR);
        BufferedReader csvData = Files.newBufferedReader(new File(Filesystem.getDeployDirectory(), CSV_DATA_PATH).getPath());

        // TODO: make this actually use the csv file

        String line;
            int lineNum = 0;

            while ((line = reader.readLine()) != null) {
                lineNum++;
                String msgPrefix = "Line #" + lineNum + ": ";
                Consumer<String> debug = (msg) -> {
                    System.out.println(msgPrefix + msg);
                };

                int hashtagIndex = line.indexOf("#");
                int doubleSlashIndex = line.indexOf("//");
                if (hashtagIndex == 0) {
                    debug.accept("Starts with '#', skipping line");
                    continue;
                }
                if (doubleSlashIndex == 0) {
                    debug.accept("Starts with '//', skipping line");
                    continue;
                }
                if (hashtagIndex != -1 && (doubleSlashIndex == -1 || hashtagIndex < doubleSlashIndex)) {
                    debug.accept("'#' at char index " + hashtagIndex + ", trimming comment");
                    line = line.substring(0, hashtagIndex);
                } else if (doubleSlashIndex != -1 && (hashtagIndex == -1 || doubleSlashIndex < hashtagIndex)) {
                    debug.accept("'//' at char index " + doubleSlashIndex + ", trimming comment");
                    line = line.substring(0, doubleSlashIndex);
                }

                String[] items = line.split(",", -1);
                if (items.length < 3) {
                    debug.accept("Less than 3 items, skipping line");
                    continue;
                } else if (items.length > 3) {
                    debug.accept("More than 3 items, ignoring extra items");
                    // Extra items aren't processed, could use Arrays.copyOf(items, [newlength]) if needed
                }

                double distance, angle, RPM;

                try {
                    distance = Double.parseDouble(items[0]);
                    angle = Double.parseDouble(items[1]);
                    RPM = Double.parseDouble(items[2]);
                } catch (NumberFormatException err) {
                    debug.accept("Non-numerical value, skipping line");
                    continue;
                }

                if (distance < 0) {
                    debug.accept("Distance " + distance + " is negative, skipping line");
                    continue;
                }
                if (angle < ShooterConstants.MIN_HOOD_ANGLE) {
                    debug.accept("Hood angle " + angle + " is less than the min value, skipping line");
                    continue;
                }
                if (angle > ShooterConstants.MAX_HOOD_ANGLE) {
                    debug.accept("Hood angle " + angle + " is greater than the max value, skipping line");
                    continue;
                }
                if (RPM < 0) {
                    debug.accept("Flywheel RPM " + RPM + " is negative, skipping line");
                    continue;
                }

                map.addDataPoint(new ShooterDataDistancePoint(distance, angle, RPM));
            }

            // Debug code
            System.out.println("All points:");
            for (ShooterDataDistancePoint point : map.values()) {
                System.out.println(point.getDistance() + ": " + point.getAngle() + ", " + point.getRPM());
            }

            System.out.println("Done deserializing CSV");
            return map;
        } catch (IOException err) {
            err.printStackTrace();
            return null;
        }

        map.put(1.0, new LauncherDataPoint(0, 0));
        map.put(10.0, new LauncherDataPoint(1.0, 2.0));

        return map;
    }

}
