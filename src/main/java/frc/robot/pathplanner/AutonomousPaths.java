package frc.robot.pathplanner;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.pathplanner.AutonomousContainer;

public class AutonomousPaths {

    private static AutonomousPaths instance;

    private final Command shootAndToss;
    private final Command intakeFromDepot;
    private final Command intakeFromFeeder;
    private final Command intakeAndShoot;
    private final Command justToss;
    private final Command justShoot;
    private final Command goAndWait;

    public static AutonomousPaths getInstance(AutonomousContainer container) {
        if (instance == null) {
            instance = new AutonomousPaths(container);
        }
        return instance;
    }

    private AutonomousPaths(AutonomousContainer container) {

        shootAndToss = new PathPlannerAuto("Shoot and Toss", container.shouldMirror());
        intakeFromDepot = new PathPlannerAuto("Intake from depot", container.shouldMirror());
        intakeFromFeeder = new PathPlannerAuto("Intake from feeder", container.shouldMirror());
        intakeAndShoot = new PathPlannerAuto("Intake and Shoot", container.shouldMirror());
        justToss = new PathPlannerAuto("Just toss", container.shouldMirror());
        justShoot = new PathPlannerAuto("Just shoot", container.shouldMirror());
        goAndWait = new PathPlannerAuto("Go and wait", container.shouldMirror());
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