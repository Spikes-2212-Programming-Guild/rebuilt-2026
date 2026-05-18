// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.spikes2212.dashboard.RootNamespace;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
//import frc.robot.commands.collection.MoveCollectionJoint;
//import frc.robot.commands.collection.SpinRoller;
//import frc.robot.commands.shooter.SimpleShoot;
//import frc.robot.commands.spindexer.SpinKicker;
//import frc.robot.commands.spindexer.SpinMagazine;
import frc.robot.commands.swerve.Drive;
import frc.robot.commands.swerve.RotateAccordingToGyro;
import frc.robot.oi.OI;
//import frc.robot.subsystems.collection.CollectionJoint;
//import frc.robot.subsystems.collection.Roller;
//import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.spindexer.Kicker;
import frc.robot.subsystems.spindexer.SpinningMagazine;
import frc.robot.subsystems.swerve.Drivetrain;

public class Robot extends TimedRobot {

    private static final RootNamespace namespace = new RootNamespace("robot");

    private final Drivetrain drivetrain = Drivetrain.getInstance(); // why not just call it swerve?
//    private final Shooter shooter = Shooter.getInstance();
//    private final Kicker kicker = Kicker.getInstance();
//    private final CollectionJoint collectionJoint = CollectionJoint.getInstance(); // not sure about the name
//    private final Roller roller = Roller.getInstance();
//    private final SpinningMagazine spinningMagazine = SpinningMagazine.getInstance();

    @Override
    public void robotInit() {
        namespace.putCommand("rotate to gyro", new RotateAccordingToGyro(drivetrain,
                namespace.addConstantDouble("gyro turn", 0.0), true));
//
//        namespace.putCommand("shoot",
//                new SimpleShoot(shooter, namespace.addConstantDouble("shoot speed", 0)));
//
//        namespace.putCommand("move collection joint",
//                new MoveCollectionJoint(collectionJoint, namespace.addConstantDouble("collection joint speed", 0)));
//
//        namespace.putCommand("spin roller", new SpinRoller(roller));
//
//        namespace.putCommand("spin kicker", new SpinKicker(kicker));
//
//        namespace.putCommand("spin magazine", new SpinMagazine(spinningMagazine));
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        namespace.update();
    }

    @Override
    public void disabledInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void disabledPeriodic() {

    }

    @Override
    public void autonomousInit() {

    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        drivetrain.resetFieldRelativity();
        drivetrain.resetRelativeEncoders();

        OI oi = new OI(namespace);
        drivetrain.setDefaultCommand(new Drive(
                drivetrain, oi::getX, oi::getY, oi::getZ, oi::useFieldRelative, true
        ));
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
}
