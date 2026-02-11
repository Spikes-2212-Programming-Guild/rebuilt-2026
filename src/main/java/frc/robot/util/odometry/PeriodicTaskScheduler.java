package frc.robot.util.odometry;

import edu.wpi.first.wpilibj.TimedRobot;

public class PeriodicTaskScheduler {

    private final TimedRobot timedRobot;

    public static PeriodicTaskScheduler instance;

    //should be in robot init

    public static void init(TimedRobot timedRobot){
        if(instance != null){
            return;
        }
        instance = new PeriodicTaskScheduler(timedRobot);
    }

    public static PeriodicTaskScheduler getInstance(){
        return instance;
    }

    private PeriodicTaskScheduler(TimedRobot timedRobot) {
        this.timedRobot = timedRobot;
    }

    public void schedule(Runnable task, double frequencyHz, double delaySeconds) {
        if (frequencyHz <= 0) {
            return;
        }
        timedRobot.addPeriodic(task, 1.0 / frequencyHz, delaySeconds);
    }
}
