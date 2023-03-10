package frc.robot.util;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

/**
 * Utilty class for logging data to the DataLogManager.
 * View logs using MechanicalAdvantage's advantage scope (https://github.com/Mechanical-Advantage/AdvantageScope)
 */
public class Logger {

  private static final DataLog log = DataLogManager.getLog();

  // These are array lists of log entry classes from WPI. appending to a log entry automatically adds to the log file.
  private static ArrayList<DoubleLogEntry> doubleLogs = new ArrayList<>();
  private static ArrayList<DoubleArrayLogEntry> doubleArrayLogs = new ArrayList<>();
  private static ArrayList<BooleanLogEntry> boolLogs = new ArrayList<>();
  private static ArrayList<IntegerLogEntry> intLogs = new ArrayList<>();

  // These are the suppliers, or functions that return values. This is how the values are accessed.
  private static ArrayList<DoubleSupplier> doubleValues = new ArrayList<>();
  private static ArrayList<DoubleSupplier[]> doubleArrayValues = new ArrayList<>();
  private static ArrayList<BooleanSupplier> boolValues = new ArrayList<>();
  private static ArrayList<IntSupplier> intValues = new ArrayList<>();

  /**
   * Starts logging a double from a function.
   * 
   * @param name The name of the log. Use / to create subdirectories, and keep names unique.
   * @param logged A double supplier of the value to be logged. Can be created by using a lambda on a function that returns a double.
   */
  public static void addDouble(String name, DoubleSupplier logged) {
    DoubleLogEntry myDoubleLog = new DoubleLogEntry(log, name);
    doubleLogs.add(myDoubleLog);
    doubleValues.add(logged);
  }

  /**
   * Starts logging an array of doubles from a function.
   * 
   * @param name The name of the log. Use / to create subdirectories, and keep names unique.
   * @param logged An array of double suppliers of the values to be logged. 
   *               Can be created by creating an array of lambdas on functions that return doubles.
   */
  public static void addDoubleArray(String name, DoubleSupplier[] logged) {
    DoubleArrayLogEntry myDoubleLog = new DoubleArrayLogEntry(log, name);
    doubleArrayLogs.add(myDoubleLog);
    doubleArrayValues.add(logged);
  }

  /**
   * Logs a int from a function.
   * 
   * @param name The name of the log. Use / to create subdirectories, and keep names unique.
   * @param logged An int supplier of the value to be logged. Can be created by using a lambda on a function that returns a int.
   */
  public static void addInt(String name, IntSupplier logged) {
    IntegerLogEntry IntegerLog = new IntegerLogEntry(log, name);
    intLogs.add(IntegerLog);
    intValues.add(logged);
  }

  /**
   * Starts logging a boolean from a function.
   * 
   * @param name The name of the log. Use / to create subdirectories, and keep names unique.
   * @param logged A boolean supplier of the value to be logged. Can be created by using a lambda on a function that returns a boolean.
   */
  public static void addBool(String name, BooleanSupplier logged) {
    BooleanLogEntry BooleanLog = new BooleanLogEntry(log, name);
    boolLogs.add(BooleanLog);
    boolValues.add(logged);
  }

  /**
   * Logs all the values that have been collected. Should be called periodically. 
   */
  public static void updateLogs() {
    for (int i = 0; i < doubleLogs.size(); i++) {
      doubleLogs.get(i).append(doubleValues.get(i).getAsDouble());
    }
    for (int i = 0; i < doubleArrayLogs.size(); i++) {
      double[] values = new double[doubleArrayValues.get(i).length];
      for (int j = 0; j < doubleArrayValues.get(i).length; j++) {
        values[j] = doubleArrayValues.get(i)[j].getAsDouble();
      }
      doubleArrayLogs.get(i).append(values);
    }
    for (int i = 0; i < intLogs.size(); i++) {
      intLogs.get(i).append(intValues.get(i).getAsInt());
    }
    for (int i = 0; i < boolLogs.size(); i++) {
      boolLogs.get(i).append(boolValues.get(i).getAsBoolean());
    }
  }
}