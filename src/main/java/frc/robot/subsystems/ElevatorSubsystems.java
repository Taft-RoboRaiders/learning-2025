package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SmartMotionConfig;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.GearBox;
import yams.mechanisms.SmartMechanism;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

public class ElevatorSubsystems extends SubsystemBase {

SparkMax elevatorL = new SparkMax(13, MotorType.kBrushless);
SparkMax elevatorR = new SparkMax(14, MotorType.kBrushless);

SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
.withMechanismCircumference(Meters.of(Inches.of(0.25).in(Meters) * 14))//sprocket curcumfrence
.withClosedLoopController(1,0,0,MetersPerSecond.of(0.2), MetersPerSecondPerSecond.of(0.5))
.withSoftLimit(Meters.of(0), Meters.of(2))
.withGearing(SmartMechanism.gearing(SmartMechanism.gearbox("7:1")))
.withIdleMode(MotorMode.BRAKE)
.withTelemetry("ElevatorMotor", TelemetryVerbosity.HIGH)
.withStatorCurrentLimit(Amps.of(40))
.withVoltageCompensation(Volts.of(12))
.withMotorInverted(false)
.withClosedLoopRampRate(Seconds.of(0.25))
.withOpenLoopRampRate(Seconds.of(0.75))
.withFeedforward(new ElevatorFeedforward(0, 0, 0))
.withControlMode(C)

}
