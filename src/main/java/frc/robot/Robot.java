// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.spikes2212.dashboard.RootNamespace;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.intake.Intake;
import frc.robot.commands.intake.MoveCollection;
import frc.robot.commands.shoot.JustShoot;
import frc.robot.commands.storage.Transport;
import frc.robot.commands.swerve.Drive;
import frc.robot.commands.swerve.RotateAccordingToGyro;
import frc.robot.subsystems.forbar.Collection;
import frc.robot.subsystems.forbar.CollectionMovement;
import frc.robot.subsystems.shoot.Shooter;
import frc.robot.subsystems.spindexer.Kicker;
import frc.robot.subsystems.swerve.Drivetrain;

public class Robot extends TimedRobot {

    private static final RootNamespace namespace = new RootNamespace("robot");

    private final Drivetrain drivetrain = Drivetrain.getInstance();
    private final Shooter shooter = Shooter.getInstance();
    private final Kicker kicker = Kicker.getInstance();
    private final CollectionMovement forbar = CollectionMovement.getInstance();
    private final Collection intake = Collection.getInstance();

    @Override
    public void robotInit() {
        namespace.putCommand("rotate gyro", new RotateAccordingToGyro(drivetrain,
                namespace.addConstantDouble("gyro turn", 0.0), true));

        namespace.putCommand("shoot",
                new JustShoot(shooter, namespace.addConstantDouble("shoot speed", 0)));

        namespace.putCommand("forbar",
                new MoveCollection(forbar, namespace.addConstantDouble("forbar speed", 0)));

        namespace.putCommand("intake", new Intake(intake));

        namespace.putCommand("kicker", new Transport(kicker));

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

        OI oi = new OI();
        drivetrain.setDefaultCommand(new Drive(drivetrain,
                oi::getControllerLeftX, oi::getControllerLeftY, oi::getControllerRightX,
                oi::isFieldRelative, true
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
