package frc.robot.util;

import java.io.File;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.FieldConstants;

/**
 * Utility class for loading paths using pathplanner.
 */
public class PathLoader {
  private static final String kTrajectoryDirectory = "pathplanner/"; // Pathplanner output folder should be src/main/deploy/pathplanner

  private static HashMap<String, List<PathPlannerTrajectory>> pathGroups = new HashMap<>();

  /**
   * Loads all of the paths in the trajectory directory (specified in the constants). 
   * These paths are loaded and stored so that they do not take time while the robot is running 
   * and can be accessed with {@link #getPathGroup() PathGroupLoader.getPathGroup()}
   */
  public static void loadPathGroups() {
    double totalTime = 0;
    File[] directoryListing = Filesystem.getDeployDirectory().toPath().resolve(kTrajectoryDirectory).toFile().listFiles();
    if (directoryListing != null) {
      for (File file : directoryListing) {
        if (file.isFile() && file.getName().indexOf(".") != -1) {
          long startTime = System.nanoTime();
          String name = file.getName().substring(0, file.getName().indexOf("."));
          pathGroups.put(name, PathPlanner.loadPathGroup(name, new PathConstraints(AutoConstants.kMaxSpeed, AutoConstants.kMaxAccel)));
          double time = (System.nanoTime() - startTime) / 1000000.0;
          totalTime += time;
          System.out.println("Processed file: " + file.getName() + ", took " + time + " milliseconds.");
        }
      }
    } else {
      System.out.println("Error processing file");
      DriverStation.reportWarning(
        "Issue with finding path files. Paths will not be loaded.",
        true
      );
    }
    System.out.println("File processing took a total of " + totalTime + " milliseconds");
  }

  /**
   * 
   * Gets a path that has already been loaded with {@link #loadPathGroups()}. The path group is a list 
   * of trajectories that path planner can run.
   * 
   * @param pathGroupName the name of the file, without any extensions. This should be the same exact name that is displayed in pathplanner
   * @return a list of trajectories that path planner can run.
   */
  public static List<PathPlannerTrajectory> getPathGroup(String pathGroupName) {
    if (pathGroups.get(pathGroupName) == null) {
      DriverStation.reportError("Error retrieving " + pathGroupName + " path!", true);
    }
    return pathGroups.get(pathGroupName);
  }

  public static PathPlannerTrajectory getPath(List<PathPlannerTrajectory> pathGroup, int pathIndex) {
    if (pathIndex < 0 || pathIndex > pathGroup.size() - 1){
      throw new IndexOutOfBoundsException("Path index out of range"); 
    } 
    return pathGroup.get(pathIndex);
  }

  public static PathPlannerState transformStateForAlliance(
      PathPlannerState state, Alliance alliance) {
    if (alliance == DriverStation.Alliance.Red) {
      // Create a new state so that we don't overwrite the original
      PathPlannerState transformedState = new PathPlannerState();

      Translation2d transformedTranslation =
          new Translation2d(FieldConstants.kLength - state.poseMeters.getX(), state.poseMeters.getY());
      Rotation2d transformedHeading = state.poseMeters.getRotation().times(-1).plus(Rotation2d.fromDegrees(180));
      Rotation2d transformedHolonomicRotation = state.holonomicRotation.times(-1).plus(Rotation2d.fromDegrees(180));

      transformedState.timeSeconds = state.timeSeconds;
      transformedState.velocityMetersPerSecond = state.velocityMetersPerSecond;
      transformedState.accelerationMetersPerSecondSq = state.accelerationMetersPerSecondSq;
      transformedState.poseMeters = new Pose2d(transformedTranslation, transformedHeading);
      transformedState.angularVelocityRadPerSec = -state.angularVelocityRadPerSec;
      transformedState.holonomicRotation = transformedHolonomicRotation;
      transformedState.holonomicAngularVelocityRadPerSec = -state.holonomicAngularVelocityRadPerSec;
      transformedState.curvatureRadPerMeter = -state.curvatureRadPerMeter;

      return transformedState;
    } else {
      return state;
    }
  }

  public static PathPlannerTrajectory transformTrajectoryForAlliance(
      PathPlannerTrajectory trajectory, Alliance alliance) {
    if (alliance == Alliance.Red) {
      List<State> transformedStates = new ArrayList<>();

      for (State s : trajectory.getStates()) {
        PathPlannerState state = (PathPlannerState) s;

        transformedStates.add(transformStateForAlliance(state, alliance));
      }

      return new PathPlannerTrajectory(
          transformedStates,
          trajectory.getMarkers(),
          trajectory.getStartStopEvent(),
          trajectory.getEndStopEvent(),
          trajectory.fromGUI);
    } else {
      return trajectory;
    }
  }

  public static Rotation2d transformYawForAlliance(Rotation2d yaw, Alliance alliance) {
    return yaw.plus(new Rotation2d(Units.degreesToRadians(alliance.equals(Alliance.Red) ? 180 : 0)));
  }
}