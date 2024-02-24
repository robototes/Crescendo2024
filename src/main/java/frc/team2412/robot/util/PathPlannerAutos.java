package frc.team2412.robot.util;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Function;

public class PathPlannerAutos {
	// Individual paths

	private static PathPlannerPath loadPathPlannerPath(String name) {
		return PathPlannerPath.fromPathFile(name);
	}

	private static PathPlannerPath loadChoreoTrajectory(String name) {
		return PathPlannerPath.fromChoreoTrajectory(name);
	}

	private static final Map<String, PathPlannerPath> pathPlannerPathCache = new HashMap<>();
	private static final Map<String, PathPlannerPath> choreoTrajectoryCache = new HashMap<>();

	public static PathPlannerPath getPathPlannerPath(String name) {
		return pathPlannerPathCache.computeIfAbsent(name, PathPlannerAutos::loadPathPlannerPath);
	}

	public static PathPlannerPath getChoreoTrajectory(String name) {
		return choreoTrajectoryCache.computeIfAbsent(name, PathPlannerAutos::loadChoreoTrajectory);
	}

	// Whole autos

	private static void addPathsFromCommand(
			JsonNode commandJson, boolean isChoreo, List<PathPlannerPath> paths) {
		String type = commandJson.get("type").asText();
		switch (type) {
			case "wait":
				break;
			case "named":
				// TODO Handle conditional paths (after those are added)
				break;
			case "path":
				String pathName = commandJson.get("data").get("pathName").asText();
				paths.add(isChoreo ? getChoreoTrajectory(pathName) : getPathPlannerPath(pathName));
				break;
			case "sequential", "parallel", "race", "deadline":
				for (JsonNode child : commandJson.get("data").get("commands")) {
					addPathsFromCommand(child, isChoreo, paths);
				}
				break;
		}
	}

	private static List<PathPlannerPath> getPaths(JsonNode autoJson) {
		JsonNode commandRoot = autoJson.get("command");
		boolean isChoreo = autoJson.get("choreoAuto").asBoolean();
		List<PathPlannerPath> paths = new ArrayList<>();
		addPathsFromCommand(commandRoot, isChoreo, paths);
		return paths;
	}

	private static Rotation2d getStartingRotation(JsonNode autoJson) {
		return Rotation2d.fromDegrees(autoJson.get("startingPose").get("rotation").asDouble());
	}

	private static ChassisSpeeds speedsFromState(PathPlannerTrajectory.State state) {
		return new ChassisSpeeds(
				state.heading.getCos() * state.velocityMps,
				state.heading.getSin() * state.velocityMps,
				state.headingAngularVelocityRps);
	}

	private static List<PathPlannerTrajectory> loadAutoTrajectories(String autoName) {
		File autoFile =
				new File(Filesystem.getDeployDirectory(), "pathplanner/autos/" + autoName + ".auto");
		if (!autoFile.exists()) {
			DriverStation.reportWarning(
					"Attempted to load non-existent auto \"" + autoName + "\"", false);
			return List.of();
		}
		JsonNode autoJson;
		try {
			autoJson = new ObjectMapper().readTree(autoFile);
		} catch (IOException e) {
			DriverStation.reportWarning("Could not load auto \"" + autoName + "\"", e.getStackTrace());
			return List.of();
		}
		List<PathPlannerPath> paths = getPaths(autoJson);
		Rotation2d startingRotation = getStartingRotation(autoJson);
		ChassisSpeeds startingSpeeds = new ChassisSpeeds();
		List<PathPlannerTrajectory> trajectories = new ArrayList<>(paths.size());
		for (var path : paths) {
			PathPlannerTrajectory trajectory = path.getTrajectory(startingSpeeds, startingRotation);
			trajectories.add(trajectory);
			startingRotation = trajectory.getEndState().targetHolonomicRotation;
			startingSpeeds = speedsFromState(trajectory.getEndState());
		}
		return List.copyOf(trajectories);
	}

	private static final Map<String, List<PathPlannerTrajectory>> autoTrajectoriesCache =
			new HashMap<>();

	public static List<PathPlannerTrajectory> getAutoTrajectories(String autoName) {
		return autoTrajectoriesCache.computeIfAbsent(autoName, PathPlannerAutos::loadAutoTrajectories);
	}

	// Pre-loading everything

	private static List<String> listFileNames(String deploySubdirectory, String extension) {
		File[] files = new File(Filesystem.getDeployDirectory(), deploySubdirectory).listFiles();

		if (files == null) {
			return List.of();
		}

		List<String> filenames = new ArrayList<>(files.length);
		for (File file : files) {
			if (!file.isDirectory()) {
				String name = file.getName();
				if (name.endsWith(extension)) {
					filenames.add(name.substring(0, name.lastIndexOf(".")));
				}
			}
		}
		return filenames;
	}

	private static <T> void loadAll(
			String deploySubdirectory,
			String extension,
			Map<String, T> cache,
			Function<String, T> loader) {
		List<String> names = listFileNames(deploySubdirectory, extension);
		for (String name : names) {
			cache.put(name, loader.apply(name));
		}
	}

	public static void loadAllPathPlannerPaths() {
		loadAll(
				"pathplanner/paths", ".path", pathPlannerPathCache, PathPlannerAutos::loadPathPlannerPath);
	}

	public static void loadAllChoreoTrajectories() {
		loadAll("choreo", ".traj", choreoTrajectoryCache, PathPlannerAutos::loadChoreoTrajectory);
	}

	public static void loadAllAutoTrajectories() {
		loadAll(
				"pathplanner/autos",
				".auto",
				autoTrajectoriesCache,
				PathPlannerAutos::loadAutoTrajectories);
	}
}
