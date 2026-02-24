package frc.robot.pathplanner;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.Command;

public class AutonomousPaths {

    private static AutonomousPaths instance;

    private final Command shootAndToss;
    private final Command intakeFromDepot;
    private final Command intakeFromFeeder;
    private final Command intakeAndShoot;
    private final Command justToss;
    private final Command justShoot;
    private final Command goAndWait;

    private static final boolean shouldMirror = AutonomousContainer.shouldMirror();

    public static AutonomousPaths getInstance() {
        if (instance == null) {
            instance = new AutonomousPaths();
        }
        return instance;
    }

    private AutonomousPaths() {
        shootAndToss = new PathPlannerAuto("Shoot and Toss", shouldMirror);
        intakeFromDepot = new PathPlannerAuto("Intake from depot", shouldMirror);
        intakeFromFeeder = new PathPlannerAuto("Intake from feeder", shouldMirror);
        intakeAndShoot = new PathPlannerAuto("Intake and Shoot", shouldMirror);
        justToss = new PathPlannerAuto("Just toss", shouldMirror);
        justShoot = new PathPlannerAuto("Just shoot", shouldMirror);
        goAndWait = new PathPlannerAuto("Go and wait", shouldMirror);
    }

    public Command getShootAndToss() {
        return shootAndToss;
    }

    public Command getIntakeFromDepot() {
        return intakeFromDepot;
    }

    public Command getIntakeFromFeeder() {
        return intakeFromFeeder;
    }

    public Command getIntakeAndShoot() {
        return intakeAndShoot;
    }

    public Command getJustToss() {
        return justToss;
    }

    public Command getJustShoot() {
        return justShoot;
    }

    public Command getGoAndWait() {
        return goAndWait;
    }
}