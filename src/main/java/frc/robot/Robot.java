// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import com.spikes2212.dashboard.RootNamespace;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.autonomous.AutonomousContainer;
import frc.robot.commands.advancedcommands.*;
import frc.robot.commands.helpers.TestSubsystems;
import frc.robot.commands.intake.SpinCollection;
import frc.robot.commands.shoot.JustShoot;
import frc.robot.commands.shoot.ShootWithPID;
import frc.robot.commands.storage.SpinMagazine;
import frc.robot.commands.storage.Transport;
import frc.robot.commands.swerve.RotateAccordingToGyro;
import frc.robot.subsystems.intake.Collection;
import frc.robot.subsystems.intake.CollectionMovement;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.spindexer.Kicker;
import frc.robot.subsystems.spindexer.SpinningMagazine;
import frc.robot.subsystems.swerve.Drivetrain;
import frc.robot.subsystems.swerve.SwerveModuleHolder;
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
        namespace.putCommand("shoot pid", new ShootWithPID(shooter, namespace.addConstantDouble("pid speed", 0), 10));
        namespace.putCommand("shoot with camera", new ShootToHub(shooter, Kicker.getInstance(), SpinningMagazine.getInstance(),
               VisionService.getInstance()));
        namespace.putNumber("distance", visionService::getZ);


        Supplier<Double> magazineSpeed = namespace.addConstantDouble("magazine speed", 0);
        Supplier<Double> transportSpeed = namespace.addConstantDouble("transport speed", 0);
        namespace.putCommand("magazine", new SpinMagazine(magazineSpeed));
        namespace.putCommand("transport", new Transport(kicker, transportSpeed));
        namespace.putCommand("move down", new MoveDown());
        namespace.putCommand("move up", new MoveUp());
        namespace.putCommand("spin collection",
                new SpinCollection(namespace.addConstantDouble("collection speed", 0.0)));
        namespace.putCommand("shoot", new JustShoot(shooter,
                namespace.addConstantDouble("shooting speed", 0.0)));
        namespace.putCommand("small up", new MoveGenericSubsystem(collectionMovement, -0.45)
                .withTimeout(namespace.addConstantDouble("up time", 0.2).get()));
        namespace.putCommand("mini jumpies", new MiniJumpies(collectionMovement));
        namespace.putCommand("gyro rotate", new RotateAccordingToGyro(drivetrain,
                namespace.addConstantDouble("gyro target", 0), true));
        namespace.putCommand("test subsystems", new TestSubsystems());
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        namespace.update();
        ShootWithPID.updateNamespace();
        drivetrain.periodic();
        SwerveModuleHolder.updateNamespace();
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
        Command auto = autonomousContainer.getSelectedCommand();
        if(auto != null){
            CommandScheduler.getInstance().schedule(auto);
        }
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
//        OI oi = new OI();
//        drivetrain.setDefaultCommand(new Drive(drivetrain, () -> oi.getLeftY() * 0.5, () -> oi.getLeftX() * 0.5,
//                () -> oi.getRightX() * -3, true, true));
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
        registerNamedCommands();
        autonomousContainer = new AutonomousContainer(drivetrain);
    }

    public void registerNamedCommands() {
        NamedCommands.registerCommand("collect and pass", new CollectAndPass());
        NamedCommands.registerCommand("collect", new Collect());
        NamedCommands.registerCommand("pass", new Shoot());
        NamedCommands.registerCommand("aligned shoot", new TuneToAprilTag(drivetrain, visionService, shooter,
                kicker, spinningMagazine, collection, 1));
//        NamedCommands.registerCommand("shoot", new JustShoot(shooter, () -> 0.3));
//        NamedCommands.registerCommand("spin", new SpinMagazine(spinningMagazine));
    }
}
