package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants;
import frc.robot.Constants.IDConstants;
import yams.mechanisms.config.ElevatorConfig;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.mechanisms.velocity.Shooter;

public class CoralSubsystem extends SubsystemBase
{
SparkMax CoralIntakeL = new SparkMax(IDConstants.CoralIntakeL_ID, MotorType.kBrushless);
SparkMax CoralIntakeR = new SparkMax(IDConstants.CoralIntakeR_ID, MotorType.kBrushless);

private final SmartMotorControllerConfig motorConfig   = new SmartMotorControllerConfig(this)
    .withStatorCurrentLimit(Amps.of(40))//
    .withVoltageCompensation(Volts.of(12))//
    .withMotorInverted(false)//
    .withClosedLoopRampRate(Seconds.of(0.25))//
    .withOpenLoopRampRate(Seconds.of(0.25))//
    .withControlMode(ControlMode.CLOSED_LOOP)//
    .withFollowers(Pair.of(CoralIntakeR, true));//

private final SmartMotorController       coralmotor         = new SparkWrapper(CoralIntakeL,
    DCMotor.getNEO(1),
    motorConfig);    

private final Shooter       motor         = new Shooter();

public CoralSubsystem()
{

}

public void periodic()
{
  coralmotor.updateTelemetry();
}

public void simulationPeriodic()
{
  coralmotor.simIterate();
}

public Command outtake(double dutycycle)
{
  return run(()-> coralmotor.setDutyCycle(dutycycle));
}

      
}
