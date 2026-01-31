// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.spikes2212.dashboard.RootNamespace;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.simplecommands.SimpleShoot;
import frc.robot.subsystems.Shooter;

import java.util.function.Supplier;

public class Robot extends TimedRobot {

    private final RootNamespace namespace = new RootNamespace("shooter");
    Shooter shoot = Shooter.getInstance();
    Supplier<Double> setpoint;
    @Override
    public void robotInit() {
        setpoint = namespace.addConstantDouble("setpoint", 0.0);
        namespace.putCommand("shoot", new SimpleShoot(shoot,setpoint));
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
