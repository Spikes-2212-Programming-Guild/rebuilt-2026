// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.spikes2212.dashboard.RootNamespace;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.autonomous.AutonomousContainer;
import frc.robot.commands.advancedcommands.*;
import frc.robot.commands.intake.SpinCollection;
import frc.robot.commands.shoot.JustShoot;
import frc.robot.commands.shoot.PIDAndBang;
import frc.robot.commands.shoot.ShootWithPID;
import frc.robot.commands.storage.SpinMagazine;
import frc.robot.commands.storage.Transport;
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

    private static final RootNamespace namespace = new RootNamespace("robot");

    private Drivetrain drivetrain;
    private Collection collection;
    private CollectionMovement collectionMovement;
    private SpinningMagazine spinningMagazine;
    private Kicker kicker;
    private Shooter shooter;

    private VisionService visionService;

    private AutonomousContainer autonomousContainer;

    @Override
    public void robotInit() {
        CommandScheduler.getInstance().cancelAll();
        initialize();
        configureDashboard();
    }

    private void configureDashboard() {
        configureShooter();
        configureCollection();
        configureSpindexer();
        namespace.putRunnable("cancel all commands", () -> CommandScheduler.getInstance().cancelAll());
    }

    private void configureSpindexer() {
        Supplier<Double> magazineSpeed = namespace.addConstantDouble("magazine speed", 0);
        Supplier<Double> transportSpeed = namespace.addConstantDouble("transport speed", 0);
        namespace.putCommand("magazine", new SpinMagazine(magazineSpeed));
        namespace.putCommand("transport", new Transport(kicker, transportSpeed));
    }

    private void configureShooter() {
        Supplier<Double> shooterSpeed = namespace.addConstantDouble("shoot speed", 0);
        namespace.putCommand("just shoot", new JustShoot(shooter, shooterSpeed));
        namespace.putCommand("pid and bang", new PIDAndBang(shooter, shooterSpeed, 1));
        namespace.putCommand("pid", new ShootWithPID(shooter, shooterSpeed, 1));
    }

    private void configureCollection() {
        Supplier<Double> collectionSpinSpeed = namespace.addConstantDouble("collection spin speed", 0);
        namespace.putCommand("spin collection", new SpinCollection(collectionSpinSpeed));

        namespace.putCommand("move up", new MoveUp());
        namespace.putCommand("move down", new MoveDown());

        namespace.putCommand("collect", new Collect());
        namespace.putCommand("jumpies", new Jumpies(CollectionMovement.getInstance()));
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        namespace.update();
        ShootWithPID.updateNamespace();
        drivetrain.periodic();
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
    }

    @Override
    public void autonomousPeriodic() {
        drivetrain.updateOdometry();
    }

    @Override
    public void teleopInit() {
        drivetrain.resetFieldRelativity();
        drivetrain.resetRelativeEncoders();
        drivetrain.resetPose(new Pose2d());
        OI oi = new OI();
        drivetrain.setDefaultCommand(new Drive(drivetrain, () -> oi.getLeftY() * 1.5, () -> oi.getLeftX() * 1.5,
                () -> oi.getRightX() * 3, true, true));
    }

    @Override
    public void teleopPeriodic() {

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
//        registerNamedCommands();
//        autonomousContainer = new AutonomousContainer(drivetrain);
    }

    public void registerNamedCommands() {
//        NamedCommands.registerCommand("collect and pass", new CollectAndPass(
//                collection, collectionMovement, shooter, () -> 0.5,
//                SpinningMagazine.getInstance(), Kicker.getInstance()));
//        NamedCommands.registerCommand("collect", new SpinCollect(collection));
//        NamedCommands.registerCommand("pass", new Pass(shooter, () -> 0.5,
//                spinningMagazine, kicker));
//        NamedCommands.registerCommand("aligned shoot", new TuneToAprilTag(drivetrain, visionService, shooter,
//                kicker, spinningMagazine, collection, 0));
//        NamedCommands.registerCommand("shoot", new JustShoot(shooter, () -> 0.3));
//        NamedCommands.registerCommand("spin", new Spin(spinningMagazine));
    }
}
