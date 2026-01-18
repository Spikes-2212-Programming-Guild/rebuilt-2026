package frc.robot.util.odometry;

import edu.wpi.first.wpilibj.TimedRobot;

public class PeriodicTaskScheduler {

    private final TimedRobot timedRobot;

    public PeriodicTaskScheduler(TimedRobot timedRobot) {
        this.timedRobot = timedRobot;
    }

    public void schedule(Runnable task, double frequencyHz, double delaySeconds) {
        if (frequencyHz <= 0) {
            return;
        }
        timedRobot.addPeriodic(task, 1.0 / frequencyHz, delaySeconds);
    }
}

