// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.spikes2212.dashboard.RootNamespace;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.advancedcommands.MoveCollectionUpSlowly;
import frc.robot.commands.intake.Intake;
import frc.robot.commands.intake.MoveCollection;
import frc.robot.commands.shoot.JustShoot;
import frc.robot.commands.shoot.PIDAndBang;
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

    private VisionService visionService;


    @Override
    public void robotInit() {
        getInstances();
        namespace.putCommand("move collection up", new MoveCollectionUpSlowly(collectionMovement));
        namespace.putCommand("move collection down", new MoveCollection(collectionMovement, () -> -0.05));
        namespace.putCommand("shoot", new ShootWithPID(shooter, namespace.addConstantDouble("shoot speed",
                -0.2), 100));
        namespace.putCommand("shoooot", new JustShoot(shooter, () -> 0.5));
        namespace.putCommand("spindexer", new Spin(spinningMagazine));
        namespace.putCommand("transport", new Transport(kicker));
        namespace.putCommand("collection", new Intake(collection));
        namespace.putCommand("b and p", new PIDAndBang(shooter, namespace.addConstantDouble("spe",
                1), 100));
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        namespace.update();
        ShootWithPID.updateNamespace();
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
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        drivetrain.resetFieldRelativity();
        drivetrain.resetRelativeEncoders();

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
}
