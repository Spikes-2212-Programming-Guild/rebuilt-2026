// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.spikes2212.dashboard.RootNamespace;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.autonomous.AutonomousContainer;
import frc.robot.autonomous.PathContainer;
import frc.robot.commands.advancedcommands.*;
import frc.robot.commands.intake.Intake;
import frc.robot.commands.intake.MoveCollection;
import frc.robot.commands.shoot.JustShoot;
import frc.robot.commands.shoot.ShootWithPID;
import frc.robot.commands.storage.Spin;
import frc.robot.commands.storage.Transport;
import frc.robot.commands.swerve.Drive;
import frc.robot.subsystems.forbar.Collection;
import frc.robot.subsystems.forbar.CollectionMovement;
import frc.robot.subsystems.shoot.Shooter;
import frc.robot.subsystems.spindexer.Kicker;
import frc.robot.subsystems.spindexer.SpinningMagazine;
import frc.robot.subsystems.swerve.Drivetrain;
import frc.robot.utils.VisionService;

public class Robot extends TimedRobot {

    private static final RootNamespace namespace = new RootNamespace("robot");

    private Drivetrain drivetrain;
    private Collection collection;
    private CollectionMovement collectionMovement;
    private SpinningMagazine spinningMagazine;
    private Kicker kicker;
    private Shooter shooter;
    private AutonomousContainer autonomousContainer;

    private VisionService visionService;

    @Override
    public void robotInit() {
        getInstances();
        registerNamedCommands();
        namespace.putCommand("move collection up", new MoveCollectionUpSlowly(collectionMovement));
        namespace.putCommand("move collection down", new MoveCollection(collectionMovement, () -> -0.15));
        namespace.putCommand("move collection up2", new MoveCollection(collectionMovement, () -> 0.5));
        namespace.putCommand("shoot", new ShootWithPID(shooter, namespace.addConstantDouble("shoot speed",
                -0.2), 100));
        namespace.putCommand("shoooot", new JustShoot(shooter, ()-> 0.1));
        namespace.putCommand("spindexer", new Spin(spinningMagazine));
        namespace.putCommand("transport", new Transport(kicker));
        namespace.putCommand("collection", new Intake(collection));
        namespace.putCommand("tune and shoot", new TuneAndShoot(shooter, kicker, spinningMagazine,
                visionService));
        autonomousContainer = new AutonomousContainer(drivetrain);
        namespace.putCommand("temp", PathContainer.getTemp());
        namespace.putRunnable("cancel all commands", () -> CommandScheduler.getInstance().cancelAll());
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
    }

    @Override
    public void disabledPeriodic() {

    }

    @Override
    public void autonomousInit() {
        drivetrain.resetFieldRelativity();
        drivetrain.resetRelativeEncoders();
//        Command autoCommand = autonomousContainer.getSelectedCommand();
//        if(!autoCommand.isScheduled()) {
//            autoCommand.execute();
//        }
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
                () -> oi.getRightX() * -3, true, true));
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

    public void getInstances() {
        drivetrain = Drivetrain.getInstance();
        collection = Collection.getInstance();
        collectionMovement = CollectionMovement.getInstance();
        spinningMagazine = SpinningMagazine.getInstance();
        kicker = Kicker.getInstance();
        shooter = Shooter.getInstance();

        visionService = VisionService.getInstance();
    }

    public void registerNamedCommands() {
        NamedCommands.registerCommand("collect and pass", new CollectAndPass(
                collection, collectionMovement, shooter, () -> 0.5,
                SpinningMagazine.getInstance(), Kicker.getInstance()));
        NamedCommands.registerCommand("collect", new Collect(collection,
                collectionMovement));
        NamedCommands.registerCommand("pass", new Pass(shooter, () -> 0.5,
                spinningMagazine, kicker));
        NamedCommands.registerCommand("aligned shoot", new TuneAndShoot(shooter,
                kicker, spinningMagazine, visionService));
        NamedCommands.registerCommand("shoot", new JustShoot(shooter, () -> 0.3));
        NamedCommands.registerCommand("spin", new Spin(spinningMagazine));
    }

}
