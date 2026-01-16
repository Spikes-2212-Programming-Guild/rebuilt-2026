package frc.robot.util.odometry;

import edu.wpi.first.wpilibj.TimedRobot;

public class PeriodicTaskScheduler {

    private static volatile PeriodicTaskScheduler instance;
    private final TimedRobot timedRobot;

    private PeriodicTaskScheduler(TimedRobot timedRobot) {
        this.timedRobot = timedRobot;
    }

    public static void init(TimedRobot timedRobot) {
        if (instance == null) {
            instance = new PeriodicTaskScheduler(timedRobot);
        }
    }

    // TODO - consider adding a throw if instance is null
    public static PeriodicTaskScheduler getInstance() {
        return instance;
    }

    public void schedule(Runnable task, double frequencyHz, double delaySeconds) {
        if (frequencyHz <= 0) {
            return;
        }
        timedRobot.addPeriodic(task, 1.0 / frequencyHz, delaySeconds);
    }
}

