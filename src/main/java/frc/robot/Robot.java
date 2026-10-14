// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.spikes2212.dashboard.RootNamespace;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.autonomous.PathContainer;
import frc.robot.commands.swerve.Drive;
import frc.robot.subsystems.swerve.Drivetrain;
import frc.robot.utils.VisionService;

public class Robot extends TimedRobot {

    public static final RootNamespace namespace = new RootNamespace("robot");
    public static final Drivetrain drivetrain = Drivetrain.getInstance();

    private VisionService visionService;

    @Override
    public void robotInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
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
        drivetrain.resetFieldRelativity();
        drivetrain.resetRelativeEncoders();
        drivetrain.resetPose(new Pose2d());

        Command auto = PathContainer.getIntakeAndShoot();
        CommandScheduler.getInstance().schedule(auto);
    }

    @Override
    public void autonomousPeriodic() {
        drivetrain.updateOdometry();
    }

    @Override
    public void teleopInit() {
        drivetrain.resetFieldRelativity();
        drivetrain.resetRelativeEncoders();

        OI oi = new OI();
        double ySpeed = 5;
        double xSpeed = 5;
        double rotationSpeed = 3;
        drivetrain.setDefaultCommand(new Drive(
                () -> squareInputs(oi.getLeftY()) * xSpeed,
                () -> squareInputs(oi.getLeftX()) * ySpeed,
                () -> squareInputs(oi.getRightX()) * rotationSpeed,
                true, true));
    }

    private static double squareInputs(double input) {
        return Math.signum(input) * (input * input);
    }

    @Override
    public void teleopPeriodic() {
        drivetrain.periodic();
        namespace.update();
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
