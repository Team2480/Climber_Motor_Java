/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.math.trajectory.constraint.MaxVelocityConstraint;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;

import static edu.wpi.first.units.Units.Value;

import java.util.ResourceBundle.Control;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType; 
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class Robot extends TimedRobot {
  private SparkMax motor;
  private SparkMaxConfig motorConfig;
  private SparkClosedLoopController closedLoopController;
  private RelativeEncoder encoder;
  private XboxController c_stick;
  private double climbStop = 0;
  private double climbGoto = 0;

  public Robot() {

    c_stick = new XboxController(0);
    /*
     * Initialize the SPARK MAX and get its encoder and closed loop controller
     * objects for later use.
     */
    motor = new SparkMax(1, MotorType.kBrushless);
    closedLoopController = motor.getClosedLoopController();
    encoder = motor.getEncoder();

    /*
     * Create a new SPARK MAX configuration object. This will store the
     * configuration parameters for the SPARK MAX that we will set below.
     */
    motorConfig = new SparkMaxConfig();

    /*
     * Configure the encoder. For this specific example, we are using the
     * integrated encoder of the NEO, and we don't need to configure it. If
     * needed, we can adjust values like the position or velocity conversion
     * factors.
     */
    motorConfig.encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1);

    /*
     * Configure the closed loop controller. We want to make sure we set the
     * feedback sensor as the primary encoder.
     */
    motorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed loop
        // slot, as it will default to slot 0.
        .p(0.1)
        .i(0)
        .d(0)
        .outputRange(-1, 1)
        // Set PID values for velocity control in slot 1
        .p(0.00005, ClosedLoopSlot.kSlot1)
        .i(0, ClosedLoopSlot.kSlot1)
        .d(0, ClosedLoopSlot.kSlot1)
        .velocityFF(.000085, ClosedLoopSlot.kSlot1)
        .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

        motorConfig.closedLoop.maxMotion
        // Set MAXMotion parameters for position control. We don't need to pass
        // a closed loop slot, as it will default to slot 0.
        .maxVelocity(3000)
        .maxAcceleration(3000)
        .allowedClosedLoopError(1)

        // Set MAXMotion parameters for velocity control in slot 1
        .maxAcceleration(8000, ClosedLoopSlot.kSlot1)
        .maxVelocity(11000, ClosedLoopSlot.kSlot1)
        .allowedClosedLoopError(1, ClosedLoopSlot.kSlot1);

        motorConfig.softLimit
        .reverseSoftLimitEnabled (true)
        .reverseSoftLimit (-50)
        .forwardSoftLimitEnabled (true)
        .forwardSoftLimit (250);

        motorConfig
        .smartCurrentLimit(50)
        .idleMode(IdleMode.kBrake);
    /*
     * Apply the configuration to the SPARK MAX.
     *
     * kResetSafeParameters is used to get the SPARK MAX to a known state. This
     * is useful in case the SPARK MAX is replaced.
     *
     * kPersistParameters is used to ensure the configuration is not lost when
     * the SPARK MAX loses power. This is useful for power cycles that may occur
     * mid-operation.
     */
    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    // Initialize dashboard values
    SmartDashboard.setDefaultNumber("Climb Position", 0);
    SmartDashboard.setDefaultNumber("Extend Position", 200);
    SmartDashboard.setDefaultNumber("Target Position", 0);
    SmartDashboard.setDefaultNumber("Goto Position", 0);
    SmartDashboard.setDefaultNumber("Stop Position", 0);
    SmartDashboard.setDefaultNumber("Target Velocity", 0);
    SmartDashboard.setDefaultBoolean("Control Mode", false);
    SmartDashboard.setDefaultBoolean("Reset Encoder", false);
    encoder.setPosition(0);
  }

  @Override
  public void teleopPeriodic() {
    if (SmartDashboard.getBoolean("Control Mode", false)) {
      /*
       * Get the target velocity from SmartDashboard and set it as the setpoint
       * for the closed loop controller.
       */
      double targetVelocity = SmartDashboard.getNumber("Target Velocity", 0);
      closedLoopController.setReference(targetVelocity, ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot1);
    } else {
      /*
       * Get the target position from SmartDashboard and set it as the setpoint
       * for the closed loop controller.
       */
      if (c_stick.getAButtonPressed()){
      double targetPosition = SmartDashboard.getNumber("Climb Position", 0);
      closedLoopController.setReference(targetPosition, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
      }
      /*This stops the climb at any point and allows for making sure the hook is tight before executing the full climb */
      if (c_stick.getAButtonReleased()){
        climbStop = encoder.getPosition();
        closedLoopController.setReference(climbStop, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
      }
/*This extends the arm to capture the cage */
      if (c_stick.getBButtonPressed()){
        double targetPosition = SmartDashboard.getNumber("Extend Position", 0);
        closedLoopController.setReference(targetPosition, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
        }
 /*This stops the extend at any point */
      if (c_stick.getBButtonReleased()){
          climbStop = encoder.getPosition();
          closedLoopController.setReference(climbStop, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
        }
         /*This alowes for minor adjustments of the climb */
         if (c_stick.getRightBumperButtonPressed()){
          climbGoto = encoder.getPosition() - 30;
          closedLoopController.setReference(climbGoto, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
        }

        /*This alowes for minor adjustments of the extend */
      if (c_stick.getLeftBumperButtonPressed()){
        climbGoto = encoder.getPosition() + 30;
        closedLoopController.setReference(climbGoto, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
      }
    }


  }

  @Override
  public void robotPeriodic() {
    // Display encoder position and velocity
    SmartDashboard.putNumber("Actual Position", encoder.getPosition());
    SmartDashboard.putNumber("Actual Velocity", encoder.getVelocity());
    SmartDashboard.putNumber("Goto Postion", climbGoto);
    SmartDashboard.putNumber("Stop Postion", climbStop);


    if (SmartDashboard.getBoolean("Reset Encoder", false)) {
      SmartDashboard.putBoolean("Reset Encoder", false);
      // Reset the encoder position to 0
      encoder.setPosition(0);
    }
  }
}