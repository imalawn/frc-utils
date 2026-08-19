package first.robot.subsystems.example;

import static first.robot.Constants.CANConstants;
import static org.wpilib.units.Units.*;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import first.robot.Constants;
import first.robot.util.io.motors.MotorIO;
import first.robot.util.io.motors.MotorIOSparkMax;
import first.robot.util.io.motors.MotorIOTalonFX;
import first.robot.util.io.motors.elevator.LinearSystem;
import first.robot.util.io.motors.elevator.LinearSystemIO;
import first.robot.util.io.motors.elevator.LinearSystemIOSim;
import first.robot.util.io.motors.pivot.Pivot;
import first.robot.util.io.motors.pivot.PivotIO;
import first.robot.util.io.motors.pivot.PivotIOSim;
import first.robot.util.io.motors.roller.Roller;
import first.robot.util.io.motors.roller.RollerIO;
import first.robot.util.io.motors.roller.RollerIOSim;
import first.robot.util.io.sensors.EncoderIOCANcoder;
import first.robot.util.subsystems.Subsystem;
import org.wpilib.math.system.DCMotor;
import org.wpilib.simulation.SingleJointedArmSim;

public class Example extends Subsystem {
  private final Roller roller;
  private final Pivot pivot;
  private final LinearSystem elevator;

  public Example() {
    /* TALON FX (Kraken/Falcon) MOTOR IO EXAMPLES */
    // Roller or flywheel with 4 total motors
    RollerIO talonRollerIO =
        switch (Constants.currentMode) {
          case REAL ->
              new MotorIOTalonFX.Builder(
                      CANConstants.EXAMPLE_CAN_BUS_0,
                      CANConstants.EXAMPLE_ROLLER_LEADER,
                      ExampleConstants.ROLLER_TALON_CONFIG)
                  .addFollower(CANConstants.EXAMPLE_ROLLER_FOLLOWER_1, MotorAlignmentValue.Aligned)
                  .addFollower(CANConstants.EXAMPLE_ROLLER_FOLLOWER_2, MotorAlignmentValue.Opposed)
                  .addFollower(CANConstants.EXAMPLE_ROLLER_FOLLOWER_3, MotorAlignmentValue.Opposed)
                  .build();
          case SIM ->
              new RollerIOSim(
                  DCMotor.getKrakenX60(4),
                  new MotorIO.RotationalMechanismConstraints(
                      ExampleConstants.ROLLER_GEAR_RATIO,
                      ExampleConstants.ROLLER_MOI,
                      ExampleConstants.ROLLER_RADIUS_METERS,
                      0,
                      0,
                      0),
                  0.3,
                  0,
                  3);
          case REPLAY -> new RollerIO() {};
        };
    // Pivot with one motor and an absolute encoder
    EncoderIOCANcoder pivotEncoder =
        new EncoderIOCANcoder(
            CANConstants.EXAMPLE_CAN_BUS_1,
            CANConstants.EXAMPLE_PIVOT_ENCODER,
            ExampleConstants.PIVOT_ENCODER_CONFIG);
    PivotIO talonPivotIO =
        switch (Constants.currentMode) {
          case REAL ->
              new MotorIOTalonFX.Builder(
                      CANConstants.EXAMPLE_CAN_BUS_1,
                      CANConstants.EXAMPLE_PIVOT_MOTOR,
                      ExampleConstants.PIVOT_TALON_CONFIG)
                  .addCANCoder(
                      pivotEncoder) /* automatically configures feedback on motor, not needed if
                                    FeedbackConfigs were manually set */
                  .addControlRequest(new MotionMagicVoltage(0)) // slot 0
                  .addControlRequest(new PositionVoltage(0)) // slot 1
                  .build();
          case SIM ->
              new PivotIOSim(
                  DCMotor.getKrakenX60(1),
                  new MotorIO.RotationalMechanismConstraints(
                      ExampleConstants.PIVOT_GEAR_RATIO,
                      SingleJointedArmSim.estimateMOI(0.5, 2),
                      0.5,
                      ExampleConstants.PIVOT_MIN_ANGLE.in(Radians),
                      ExampleConstants.PIVOT_MAX_ANGLE.in(Radians),
                      0),
                  60.0,
                  0.0,
                  0);
          case REPLAY -> new PivotIO() {};
        };
    // Elevator with one follower motor
    LinearSystemIO talonElevatorIO =
        switch (Constants.currentMode) {
          case REAL ->
              new MotorIOTalonFX.Builder(
                      CANConstants.EXAMPLE_CAN_BUS_0,
                      CANConstants.EXAMPLE_ELEVATOR_LEADER,
                      ExampleConstants.ELEVATOR_TALON_CONFIG)
                  .addFollower(CANConstants.EXAMPLE_ELEVATOR_FOLLOWER, MotorAlignmentValue.Aligned)
                  .addControlRequest(new MotionMagicVoltage(0))
                  .build();
          case SIM ->
              new LinearSystemIOSim(
                  DCMotor.getKrakenX60(2),
                  new MotorIO.LinearMechanismConstraints(
                      ExampleConstants.ELEVATOR_GEAR_RATIO,
                      ExampleConstants.ELEVATOR_LOAD_KG,
                      ExampleConstants.ELEVATOR_DRUM_RADIUS,
                      ExampleConstants.ELEVATOR_MIN_HEIGHT.in(Meters),
                      ExampleConstants.ELEVATOR_MAX_HEIGHT.in(Meters)),
                  60.0,
                  0.0,
                  1);
          case REPLAY -> new LinearSystemIO() {};
        };

    /* SPARK MAX (NEO/other) MOTOR IO EXAMPLES */
    // Roller or flywheel with 4 total motors
    RollerIO sparkRollerIO =
        switch (Constants.currentMode) {
          case REAL ->
              new MotorIOSparkMax(
                  0,
                  CANConstants.EXAMPLE_ROLLER_LEADER,
                  ExampleConstants.ROLLER_SPARK_CONFIG,
                  Rotations, // encoder position units, Rotations by default
                  RotationsPerSecond, // encoder velocity units, RotationsPerMinute by default
                  new int[] {
                    CANConstants.EXAMPLE_ROLLER_FOLLOWER_1,
                    CANConstants.EXAMPLE_ROLLER_FOLLOWER_2,
                    CANConstants.EXAMPLE_ROLLER_FOLLOWER_3
                  },
                  new boolean[] {false, true, true});
          case SIM ->
              new RollerIOSim(
                  DCMotor.getNEO(4),
                  new MotorIO.RotationalMechanismConstraints(
                      ExampleConstants.ROLLER_GEAR_RATIO,
                      ExampleConstants.ROLLER_MOI,
                      ExampleConstants.ROLLER_RADIUS_METERS,
                      0,
                      0,
                      0),
                  0.3,
                  0,
                  3);
          case REPLAY -> new RollerIO() {};
        };
    // Pivot with one motor and an absolute encoder
    PivotIO sparkPivotIO =
        switch (Constants.currentMode) {
          case REAL ->
              new MotorIOSparkMax(
                  1,
                  CANConstants.EXAMPLE_PIVOT_MOTOR,
                  ExampleConstants.PIVOT_SPARK_CONFIG,
                  Degrees,
                  RotationsPerMinute);
          case SIM ->
              new PivotIOSim(
                  DCMotor.getNEO(1),
                  new MotorIO.RotationalMechanismConstraints(
                      ExampleConstants.PIVOT_GEAR_RATIO,
                      SingleJointedArmSim.estimateMOI(0.5, 2),
                      0.5,
                      ExampleConstants.PIVOT_MIN_ANGLE.in(Radians),
                      ExampleConstants.PIVOT_MAX_ANGLE.in(Radians),
                      0),
                  60.0,
                  0.0,
                  0);
          case REPLAY -> new PivotIO() {};
        };
    // Elevator with one follower motor
    LinearSystemIO sparkElevatorIO =
        switch (Constants.currentMode) {
          case REAL ->
              new MotorIOSparkMax(
                  0,
                  CANConstants.EXAMPLE_ELEVATOR_LEADER,
                  ExampleConstants.ELEVATOR_SPARK_CONFIG,
                  Radians,
                  RadiansPerSecond,
                  new int[] {CANConstants.EXAMPLE_ELEVATOR_FOLLOWER},
                  new boolean[] {false});
          case SIM ->
              new LinearSystemIOSim(
                  DCMotor.getNEO(2),
                  new MotorIO.LinearMechanismConstraints(
                      ExampleConstants.ELEVATOR_GEAR_RATIO,
                      ExampleConstants.ELEVATOR_LOAD_KG,
                      ExampleConstants.ELEVATOR_DRUM_RADIUS,
                      ExampleConstants.ELEVATOR_MIN_HEIGHT.in(Meters),
                      ExampleConstants.ELEVATOR_MAX_HEIGHT.in(Meters)),
                  60.0,
                  0.0,
                  1);
          case REPLAY -> new LinearSystemIO() {};
        };

    // pass in the motor io objects to their respective mechanisms to finish
    // this works the same no matter the motor controller type
    roller = new Roller("ExampleRoller", talonRollerIO);
    pivot = new Pivot("ExamplePivot", talonPivotIO, pivotEncoder);
    elevator =
        new LinearSystem.Builder("ExampleLinearSystem", talonElevatorIO)
            .setDrumRadius(ExampleConstants.ELEVATOR_DRUM_RADIUS)
            .build();

    // if you wanted to use a SPARK MAX with a CANcoder, you would have to 'seed' the motor's
    // built-in encoder using the CANcoder and then use the built-in encoder for everything else.
    // Example:
    Pivot sparkPivot =
        new Pivot("SparkMaxPivotWithCANCoder", sparkPivotIO, pivotEncoder); // (your pivot object)
    sparkPivot.resetPosition(sparkPivot.getAbsolutePosition(true)); // put at end of constructor
  }
}
