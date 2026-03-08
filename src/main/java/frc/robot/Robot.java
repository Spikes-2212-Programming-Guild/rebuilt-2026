// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import com.spikes2212.dashboard.RootNamespace;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.advancedcommands.MoveCollectionUpSlowly;
import frc.robot.commands.difficultcommands.Drive;
import frc.robot.subsystems.swerve.Drivetrain;
import frc.robot.subsystems.swerve.SwerveModuleHolder;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.advancedcommands.Collect;
import frc.robot.commands.advancedcommands.UpSlowly;
import frc.robot.commands.simplecommands.MoveCollection;
import frc.robot.commands.simplecommands.SimpleIntake;
import frc.robot.commands.simplecommands.SimpleShoot;
import frc.robot.subsystems.Collection;
import frc.robot.subsystems.CollectionMovement;
import frc.robot.subsystems.Shooter;

public class Robot extends TimedRobot {

    Drivetrain drivetrain = Drivetrain.getInstance();

    private static final RootNamespace namespace = new RootNamespace("robot");
    private final Shooter shooter = Shooter.getInstance();
    private final CollectionMovement collectionMovement = CollectionMovement.getInstance();
    private final Collection collection = Collection.getInstance();

    @Override
    public void robotInit() {

    }

    @Override
    public void robotPeriodic() {
        SwerveModuleHolder.updateNamespace();
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
}
