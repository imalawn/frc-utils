package first.robot.subsystems.example;

import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Rotations;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.*;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.*;
import org.wpilib.math.util.Units;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Distance;

public class ExampleConstants {
  public static final double ROLLER_GEAR_RATIO = 1.0;
  public static final double ROLLER_MOI = 0.01;
  public static final double ROLLER_RADIUS_METERS = Units.inchesToMeters(4.0);

  public static final TalonFXConfiguration ROLLER_TALON_CONFIG =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(60)
                  .withSupplyCurrentLimit(40)
                  .withStatorCurrentLimitEnable(true)
                  .withSupplyCurrentLimitEnable(true))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(InvertedValue.CounterClockwise_Positive)
                  .withNeutralMode(NeutralModeValue.Coast))
          .withSlot0(new Slot0Configs().withKP(0.3).withKI(0).withKD(0).withKS(0).withKV(0.12));

  public static final SparkBaseConfig ROLLER_SPARK_CONFIG =
      new SparkMaxConfig()
          .smartCurrentLimit(50)
          .voltageCompensation(12.0)
          .inverted(false)
          .idleMode(SparkBaseConfig.IdleMode.kCoast)
          .apply(
              new ClosedLoopConfig()
                  .pid(2.0, 0.0, 0.03)
                  .apply(new FeedForwardConfig().kS(0.05).kV(0.12)))
          .apply(new EncoderConfig().velocityConversionFactor(1.0 / 60)); // convert to rps

  public static final double PIVOT_GEAR_RATIO = 75;
  public static final Angle PIVOT_MIN_ANGLE = Rotations.of(0);
  public static final Angle PIVOT_MAX_ANGLE = Rotations.of(130);

  public static final TalonFXConfiguration PIVOT_TALON_CONFIG =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(80)
                  .withSupplyCurrentLimit(60)
                  .withStatorCurrentLimitEnable(true)
                  .withSupplyCurrentLimitEnable(true))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(InvertedValue.CounterClockwise_Positive)
                  .withNeutralMode(NeutralModeValue.Brake))
          .withFeedback(
              new FeedbackConfigs()
                  .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
                  .withSensorToMechanismRatio(PIVOT_GEAR_RATIO))
          .withSlot0(
              new Slot0Configs()
                  .withKP(60)
                  .withKI(0)
                  .withKD(0)
                  .withKS(0.05)
                  .withKV(0.12)
                  .withKG(0)
                  .withGravityType(GravityTypeValue.Arm_Cosine))
          .withMotionMagic(
              new MotionMagicConfigs()
                  .withMotionMagicCruiseVelocity(10)
                  .withMotionMagicAcceleration(10));

  public static final SparkBaseConfig PIVOT_SPARK_CONFIG =
      new SparkMaxConfig()
          .smartCurrentLimit(60)
          .voltageCompensation(12.0)
          .inverted(false)
          .idleMode(SparkBaseConfig.IdleMode.kBrake)
          .apply(
              new ClosedLoopConfig()
                  .pid(2.0, 0.0, 0.03)
                  .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                  .apply(
                      new FeedForwardConfig().kS(0.05).kV(0.12).kCos(0.0).kCosRatio(1.0 / 360.0)))
          .apply(
              /* this motor will use the absolute encoder attached to
              the SPARK MAX data port */
              new AbsoluteEncoderConfig()
                  .positionConversionFactor(360.0 / PIVOT_GEAR_RATIO) // convert to deg
                  .inverted(false)
                  .zeroOffset(0.102839)
                  .zeroCentered(false));

  public static final CANcoderConfiguration PIVOT_ENCODER_CONFIG =
      new CANcoderConfiguration()
          .withMagnetSensor(
              new MagnetSensorConfigs()
                  .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
                  .withMagnetOffset(Rotations.of(0.1845703125))
                  .withAbsoluteSensorDiscontinuityPoint(0.7));

  public static final double ELEVATOR_GEAR_RATIO = 1.68;
  public static final double ELEVATOR_DRUM_RADIUS = 0.1; // meters
  public static final double ELEVATOR_LOAD_KG = 2.0;
  public static final Distance ELEVATOR_MIN_HEIGHT = Meters.of(0);
  public static final Distance ELEVATOR_MAX_HEIGHT = Meters.of(2);

  public static final TalonFXConfiguration ELEVATOR_TALON_CONFIG =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(80)
                  .withSupplyCurrentLimit(60)
                  .withStatorCurrentLimitEnable(true)
                  .withSupplyCurrentLimitEnable(true))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(InvertedValue.CounterClockwise_Positive)
                  .withNeutralMode(NeutralModeValue.Brake))
          .withFeedback(
              new FeedbackConfigs()
                  .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                  .withSensorToMechanismRatio(ELEVATOR_GEAR_RATIO))
          .withSlot0(
              new Slot0Configs()
                  .withKP(68.157)
                  .withKI(0)
                  .withKD(0.179)
                  .withKS(0.1)
                  .withKV(4.243)
                  .withKA(0.011)
                  .withKG(0.109)
                  .withGravityType(GravityTypeValue.Elevator_Static))
          .withMotionMagic(
              new MotionMagicConfigs()
                  .withMotionMagicCruiseVelocity(3)
                  .withMotionMagicAcceleration(80));

  public static final SparkBaseConfig ELEVATOR_SPARK_CONFIG =
      new SparkMaxConfig()
          .smartCurrentLimit(60)
          .voltageCompensation(12.0)
          .inverted(false)
          .idleMode(SparkBaseConfig.IdleMode.kBrake)
          .apply(
              new ClosedLoopConfig()
                  .pid(2.0, 0.0, 0.03)
                  .apply(new FeedForwardConfig().kS(0.05).kV(0.12).kG(0.0)))
          .apply(
              new EncoderConfig()
                  .positionConversionFactor(Math.PI / ELEVATOR_GEAR_RATIO) // convert to rads
                  .velocityConversionFactor(
                      Math.PI / ELEVATOR_GEAR_RATIO / 60.0)); // convert to rads/sec
}
