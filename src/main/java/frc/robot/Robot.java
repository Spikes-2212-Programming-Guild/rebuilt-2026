// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.spikes2212.dashboard.RootNamespace;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.advancedcommands.Pass;
import frc.robot.commands.advancedcommands.TrenchShoot;
import frc.robot.commands.advancedcommands.ShootToHub;
import frc.robot.commands.autonomous.DriveAndShoot;
import frc.robot.commands.shooter.ShootWithPID;
import frc.robot.commands.shooter.SimpleShoot;
import frc.robot.commands.swerve.Drive;
import frc.robot.subsystems.intake.Collection;
import frc.robot.subsystems.intake.CollectionMovement;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.spindexer.Kicker;
import frc.robot.subsystems.spindexer.SpinningMagazine;
import frc.robot.subsystems.swerve.Drivetrain;
import frc.robot.utils.VisionService;

import java.util.function.Supplier;

public class Robot extends TimedRobot {

    public static final RootNamespace namespace = new RootNamespace("robot");

    private Drivetrain drivetrain;
    private Collection collection;
    private CollectionMovement collectionMovement;
    private SpinningMagazine spinningMagazine;
    private Kicker kicker;
    private Shooter shooter;

    private VisionService visionService;

//    private AutonomousContainer autonomousContainer;

    @Override
    public void robotInit() {
        CommandScheduler.getInstance().cancelAll();
        initialize();
        Supplier<Double> speed = namespace.addConstantDouble("shooter speed!", 0);
        namespace.putCommand("shoot with pid123", new ShootWithPID(shooter, speed, 0.75));


        namespace.putCommand("trench shoot", new TrenchShoot());
        namespace.putCommand("simple shoot", new SimpleShoot(speed));
        Supplier<Double> speedAnother = namespace.addConstantDouble("shoot another thingy", 0);
        namespace.putCommand("shoot to hub", new ShootToHub(speedAnother));
        namespace.putNumber("x", () -> drivetrain.getDistanceFromHub());
        namespace.putNumber("z", () -> drivetrain.getAngleFromHub());
        namespace.putNumber("gyro % 360", () -> drivetrain.getAngle().getDegrees() % 360);
        namespace.putCommand("pass command", new Pass());
        namespace.putCommand("simple auto", new DriveAndShoot(drivetrain));
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
        CommandScheduler.getInstance().cancelAll();
        shooter.stop();
        collectionMovement.setCoast();
    }

    @Override
    public void disabledPeriodic() {

    }

    @Override
    public void autonomousInit() {
        drivetrain.resetFieldRelativity();
        drivetrain.resetRelativeEncoders();
        drivetrain.resetPose(new Pose2d());

        Command auto = new DriveAndShoot(drivetrain);
        CommandScheduler.getInstance().schedule(auto);
    }

    @Override
    public void autonomousPeriodic() {
//        drivetrain.updateOdometry();
    }

    @Override
    public void teleopInit() {
        drivetrain.resetFieldRelativity();
        drivetrain.resetRelativeEncoders();
        drivetrain.resetPose(new Pose2d());

        OI oi = new OI();
        double ySpeed = 2;
        double xSpeed = 2;
        double rotationSpeed = 2;
        drivetrain.setDefaultCommand(new Drive(drivetrain,
                () -> squareInputs(oi.getLeftY() * xSpeed),
                () -> squareInputs(oi.getLeftX() * ySpeed),
                () -> squareInputs(oi.getRightX() * rotationSpeed),
                true, true));
    }

    private static double squareInputs(double input) {
        return Math.signum(input) * (input * input);
    }

    @Override
    public void teleopPeriodic() {
//        ShootWithPID.updateNamespace();
        namespace.update();
        drivetrain.periodic();
        ShootWithPID.updateNamespace();
//        ShootWithPID.updateNamespace();
//        namespace.update();
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {

    }

    @Override
    public void simulationInit() {

    }

    @Override
    public void simulationPeriodic() {

    }

    public void initialize() {
        drivetrain = Drivetrain.getInstance();
        collection = Collection.getInstance();
        collectionMovement = CollectionMovement.getInstance();
        spinningMagazine = SpinningMagazine.getInstance();
        kicker = Kicker.getInstance();
        shooter = Shooter.getInstance();
        visionService = VisionService.getInstance();
    }

//    public void registerNamedCommands(){
//        NamedCommands.registerCommand("collect", new Collect());
//        NamedCommands.registerCommand("aligned shoot", new ShootToTag());
//        NamedCommands.registerCommand("collect and pass", new CollectAndPass());
//        NamedCommands.registerCommand("");
//    }
}
