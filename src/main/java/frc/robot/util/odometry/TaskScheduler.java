package frc.robot.util.odometry;

import edu.wpi.first.wpilibj.TimedRobot;

public class TaskScheduler {

    private final TimedRobot timedRobot;

    public TaskScheduler(TimedRobot timedRobot) {
        this.timedRobot = timedRobot;
    }

    public void schedule(Runnable task, double frequencyHz, double offsetSeconds) {
        if (frequencyHz <= 0) {
            return;
        }
        timedRobot.addPeriodic(task, 1.0 / frequencyHz, offsetSeconds);
    }
}

