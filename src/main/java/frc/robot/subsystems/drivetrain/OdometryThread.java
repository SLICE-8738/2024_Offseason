package frc.robot.subsystems.drivetrain;

import edu.wpi.first.wpilibj.Notifier;

import frc.robot.Constants;

import java.util.ArrayList;
import java.util.List;
import java.util.OptionalDouble;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

/**
 * Provides an interface for asynchronously reading high-frequency measurements to a set of queues.
 */
public class OdometryThread {
  private List<Supplier<OptionalDouble>> signals = new ArrayList<>();
  private List<Queue<Double>> queues = new ArrayList<>();
  private List<Queue<Double>> timestampQueues = new ArrayList<>();

  private final Notifier notifier;
  private static OdometryThread instance = null;

  public static OdometryThread getInstance() {
    if (instance == null) {
      instance = new OdometryThread();
    }
    return instance;
  }

  private OdometryThread() {
    notifier = new Notifier(this::periodic);
    notifier.setName("OdometryThread");
  }

  public void start() {
    if (timestampQueues.size() > 0) {
      notifier.startPeriodic(1.0 / Constants.kDrivetrain.ODOMETRY_FREQUENCY_HZ);
    }
  }

  public Queue<Double> registerSignal(Supplier<OptionalDouble> signal) {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    Drivetrain.lockOdometry();
    try {
      signals.add(signal);
      queues.add(queue);
    } finally {
      Drivetrain.unlockOdometry();
    }
    return queue;
  }

  public Queue<Double> makeTimestampQueue() {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    Drivetrain.lockOdometry();
    try {
      timestampQueues.add(queue);
    } finally {
      Drivetrain.unlockOdometry();
    }
    return queue;
  }

  private void periodic() {
    Drivetrain.lockOdometry();
    double timestamp = Logger.getRealTimestamp() / 1e6;
    try {
      double[] values = new double[signals.size()];
      boolean isValid = true;
      for (int i = 0; i < signals.size(); i++) {
        OptionalDouble value = signals.get(i).get();
        if (value.isPresent()) {
          values[i] = value.getAsDouble();
        } else {
          isValid = false;
          break;
        }
      }
      if (isValid) {
        for (int i = 0; i < queues.size(); i++) {
          queues.get(i).offer(values[i]);
        }
        for (int i = 0; i < timestampQueues.size(); i++) {
          timestampQueues.get(i).offer(timestamp);
        }
      }
    } finally {
      Drivetrain.unlockOdometry();
    }
  }
}