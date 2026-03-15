package frc.robot.autonomous;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.autonomous.autoincode.GoAndWaitAuto;
import frc.robot.autonomous.autoincode.JustShootAuto;
import frc.robot.subsystems.shoot.Shooter;
import frc.robot.subsystems.spindexer.Kicker;
import frc.robot.subsystems.spindexer.SpinningMagazine;
import frc.robot.subsystems.swerve.Drivetrain;
import frc.robot.utils.VisionService;

public class PathContainer {

    private static PathContainer instance;

    private final Command shootAndToss;
    private final Command intakeFromDepot;
    private final Command intakeFromFeeder;
    private final Command intakeAndShoot;
    private final Command justToss;
    private final Command justShoot;
    private final Command goAndWait;

    private static final boolean shouldMirror = AutonomousContainer.shouldMirror();

    public static PathContainer getInstance() {
        if (instance == null) {
            instance = new PathContainer();
        }
        return instance;
    }

    private PathContainer() {
        shootAndToss = new PathPlannerAuto("Shoot and Toss", shouldMirror);
        intakeFromDepot = new PathPlannerAuto("Intake from depot", shouldMirror);
        intakeFromFeeder = new PathPlannerAuto("Intake from feeder", shouldMirror);
        intakeAndShoot = new PathPlannerAuto("Intake and Shoot", shouldMirror);
        justToss = new PathPlannerAuto("Just toss", shouldMirror);
        justShoot = new PathPlannerAuto(new JustShootAuto(Drivetrain.getInstance(), Shooter.getInstance(),
                Kicker.getInstance(), SpinningMagazine.getInstance(), VisionService.getInstance()));
        goAndWait = new PathPlannerAuto(new GoAndWaitAuto(Drivetrain.getInstance()));
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