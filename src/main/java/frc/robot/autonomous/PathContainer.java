package frc.robot.autonomous;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.autonomous.autoincode.GoAndWaitAuto;
import frc.robot.autonomous.autoincode.JustShootAuto;
import frc.robot.subsystems.intake.Collection;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.spindexer.Kicker;
import frc.robot.subsystems.spindexer.SpinningMagazine;
import frc.robot.subsystems.swerve.Drivetrain;
import frc.robot.utils.VisionService;

public class PathContainer {

    private static PathContainer instance;

    private static Command FilppedShootAndToss;
    private static Command intakeFromDepot;
    private static Command intakeFromFeeder;
    private static Command intakeAndShoot;
    private static Command shootAndPass;
    private static Command justShoot;
    private static Command goAndWait;
    private static Command temp;

    private static final boolean shouldMirror = AutonomousContainer.shouldMirror();

    public static void createAutos() {
        FilppedShootAndToss = new PathPlannerAuto("Flipped shoot and pass", shouldMirror);
        intakeFromDepot = new PathPlannerAuto("Intake from depot", shouldMirror);
        intakeFromFeeder = new PathPlannerAuto("Intake from feeder", shouldMirror);
        intakeAndShoot = new PathPlannerAuto("Intake and Shoot", shouldMirror);
        shootAndPass = new PathPlannerAuto("Shoot and pass", shouldMirror);
        justShoot = new PathPlannerAuto(new JustShootAuto(Drivetrain.getInstance(), Shooter.getInstance(),
                Kicker.getInstance(), SpinningMagazine.getInstance(), VisionService.getInstance(),
                Collection.getInstance(), () -> 0.0));
        goAndWait = new PathPlannerAuto(new GoAndWaitAuto(Drivetrain.getInstance()));
        temp = new PathPlannerAuto("temp", shouldMirror);
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
        return FilppedShootAndToss;
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

    public static Command getJustShoot() {
        return justShoot;
    }

    public static Command getGoAndWait() {
        return goAndWait;
    }

    public static Command getTemp() {
        return temp;
    }
}