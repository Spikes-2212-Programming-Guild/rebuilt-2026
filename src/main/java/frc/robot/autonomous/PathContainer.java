package frc.robot.autonomous;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.autonomous.DriveAndShoot;
import frc.robot.subsystems.intake.Collection;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.spindexer.Kicker;
import frc.robot.subsystems.spindexer.SpinningMagazine;
import frc.robot.subsystems.swerve.Drivetrain;
import frc.robot.utils.VisionService;

public class PathContainer {

    private static PathContainer instance;

    private static Command flippedShootAndToss;
    private static Command intakeFromDepot;
    private static Command intakeFromFeeder;
    private static Command intakeAndShoot;
    private static Command shootAndPass;
    private static Command flippedCollectAndShoot;
    private static Command collectAndShoot;
    private static Command justShoot;
    private static Command temp;
    private static Command something;


    private static final boolean shouldMirror = AutonomousContainer.shouldMirror();

    public static void createAutos() {
        flippedShootAndToss = new PathPlannerAuto("Flipped shoot and pass", shouldMirror);
        intakeFromDepot = new PathPlannerAuto("Intake from depot", shouldMirror);
        intakeFromFeeder = new PathPlannerAuto("Intake from feeder", shouldMirror);
        intakeAndShoot = new PathPlannerAuto("Intake and Shoot", shouldMirror);
        shootAndPass = new PathPlannerAuto("Shoot and pass", shouldMirror);
        flippedCollectAndShoot = new PathPlannerAuto("flipped collect and shoot", shouldMirror);
        collectAndShoot = new PathPlannerAuto("collect and shoot", shouldMirror);
        justShoot = new PathPlannerAuto(new DriveAndShoot(Drivetrain.getInstance(), Shooter.getInstance(),
                Kicker.getInstance(), SpinningMagazine.getInstance(), VisionService.getInstance(),
                Collection.getInstance()));
        temp = new PathPlannerAuto("temp", shouldMirror);
        something = new PathPlannerAuto("something", shouldMirror);
    }


    public static PathContainer getInstance() {
        if (instance == null) {
            instance = new PathContainer();
        }
        return instance;
    }

    private PathContainer() {

    }

    public static Command getFlippedShootAndPass() {
        return flippedShootAndToss;
    }

    public static Command getIntakeFromDepot() {
        return intakeFromDepot;
    }

    public static Command getIntakeFromFeeder() {
        return intakeFromFeeder;
    }

    public static Command getIntakeAndShoot() {
        return intakeAndShoot;
    }

    public static Command getShootAndPass() {
        return shootAndPass;
    }

    public static Command getFlippedCollectAndShoot() {
        return flippedCollectAndShoot;
    }

    public static Command getCollectAndShoot() {
        return collectAndShoot;
    }

    public static Command getJustShoot() {
        return justShoot;
    }

    public static Command getTemp() {
        return temp;
    }

    public static Command getSomething() {
        return something;
    }
}