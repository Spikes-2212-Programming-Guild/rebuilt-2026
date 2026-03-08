// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.spikes2212.dashboard.RootNamespace;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.swerve.Drive;
import frc.robot.subsystems.forbar.Collection;
import frc.robot.subsystems.forbar.CollectionMovement;
import frc.robot.subsystems.shoot.Shooter;
import frc.robot.subsystems.spindexer.SpinningMagazine;
import frc.robot.subsystems.spindexer.Transport;
import frc.robot.subsystems.swerve.Drivetrain;
import frc.robot.subsystems.swerve.SwerveModuleHolder;
import frc.robot.utils.VisionService;

public class Robot extends TimedRobot {

    private static final RootNamespace namespace = new RootNamespace("robot");

    private Drivetrain drivetrain;
    private Collection collection;
    private CollectionMovement collectionMovement;
    private SpinningMagazine spinningMagazine;
    private Transport transport;
    private Shooter shooter;

    VisionService visionService;


    @Override
    public void robotInit() {
        getInstances();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        SwerveModuleHolder.updateNamespace();
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
        drivetrain.setDefaultCommand(new Drive(drivetrain, () -> oi.getLeftY() * 2, () -> oi.getLeftX() * 2,
                () -> oi.getRightX() * -0.6, true, false));
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
        transport = Transport.getInstance();
        shooter = Shooter.getInstance();

        visionService = VisionService.getInstance();
    }
}
