// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.config.SparkMaxConfig;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.SparkBase.PersistMode;
// import com.revrobotics.spark.SparkBase.ResetMode;


/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
 

 


  //private XboxController m_leftTrigger;
 
  private final XboxController m_XboxController = new XboxController(0);


  private final Timer m_Timer = new Timer();

  private final Spark m_leftFrontMotor = new Spark(2);
  private final Spark m_leftBackMotor = new Spark(0);
  private final Spark m_rightBackMotor = new Spark(5);
  private final Spark m_rightFrontMotor = new Spark(3);


  private final Spark m_elevator = new Spark(4);
  private final Spark m_claw = new Spark(1);
 



  private final Encoder m_elevatorEncoder = new Encoder(1, 2);
  private final Encoder m_clawEncoder = new Encoder(5, 6);




  private final MecanumDrive m_MecanumDrive = new MecanumDrive(m_leftFrontMotor, m_leftBackMotor, m_rightFrontMotor, m_rightBackMotor);
 
  private final SendableChooser<String> chooser = new SendableChooser<>();
  private String Starting_place;

 
  private final String Left = "Left";
  private final String Right = "Right";

  public double TurnK;

  public double kP_elevator = 0.4;
  public double kI_elevator = 1; // start with 0.4
  public double kD_elevator = 0.1; // start with 0.1
  public double errorSum_elevator = 0;
  public double lastTimestamp_elevator = 0;
  public double lastError_elevator = 0;
  public double iLimit_elevator = 1/2;
  public double ElevatorPosition, dT_elevator, sensorPosition_elevator, error_elevator, outputSpeed_elevator, errorRate_elevator;


 
  public double kP_claw = -0.4;
  public double ClawPosition, sensorPosition_claw, error_claw, outputSpeed_claw;



  @Override
  public void autonomousInit() {


    m_elevatorEncoder.reset();
    m_clawEncoder.reset();

    errorSum_elevator = 0;
    lastError_elevator = 0;
    ElevatorPosition = 0;

    Starting_place = chooser.getSelected();
   
    m_Timer.reset();
    m_Timer.start();

    lastTimestamp_elevator = m_Timer.getFPGATimestamp();

    if (Starting_place.equals(Left)) {
      TurnK = 1;
    } else if (Starting_place.equals(Right)) {
      TurnK = -1;
    } else {
      //bhdfhfd
    }
   
  }
  @Override
  public void autonomousPeriodic() {


    //(forward, turn, side to side)
   
    if (m_Timer.get() < 1) {
      m_MecanumDrive.driveCartesian(0, 0, 0);
      ElevatorPosition = 0;
    } else if(m_Timer.get() < 1.25) {
      m_MecanumDrive.driveCartesian(1, 0, 0);
    } else if (m_Timer.get() < 1.5) {
      m_MecanumDrive.driveCartesian(0, (1*TurnK), 0);
    } else if (m_Timer.get() < 2.55){
      m_MecanumDrive.driveCartesian(1, 0, 0);
    } else if (m_Timer.get() < 3){
      ElevatorPosition = 1.2083333;
    } else if (m_Timer.get() < 4){
      // claw
    } else if (m_Timer.get() < 4.25){
      m_MecanumDrive.driveCartesian(-1, 0, 0);
      ElevatorPosition = 0;
    }  else if (m_Timer.get() < 4.45){
      m_MecanumDrive.driveCartesian(0, (-1*TurnK), 0);
    }  else if (m_Timer.get() < 5.65){
      m_MecanumDrive.driveCartesian(1, 0, 0);
    } else {
      //jhdfhgdfhg
    }

       
    sensorPosition_elevator = m_elevatorEncoder.getDistance() / 4020;
    error_elevator = ElevatorPosition - sensorPosition_elevator;
    dT_elevator = m_Timer.getFPGATimestamp() - lastTimestamp_elevator;

    if (Math.abs(error_elevator) < iLimit_elevator){
      errorSum_elevator += error_elevator * dT_elevator;
    }

    errorRate_elevator = (error_elevator - lastError_elevator) / dT_elevator;

    outputSpeed_elevator = kP_elevator * error_elevator + kI_elevator * errorSum_elevator + kD_elevator * errorRate_elevator;

    //m_elevator.set(outputSpeed_elevator);

    lastTimestamp_elevator = m_Timer.getFPGATimestamp();
    lastError_elevator = error_elevator;










    sensorPosition_claw = m_clawEncoder.getDistance() / 1; // find variable
    error_claw = ClawPosition - sensorPosition_claw;

    //outputSpeed_claw = kP_claw * error_claw;

    //m_claw.set(outputSpeed_claw);

  }




  @Override
  public void robotInit() {




    chooser.setDefaultOption("Left", Left);
    chooser.addOption("Right", Right);
    SmartDashboard.putData("Auto choices", chooser);


    SendableRegistry.addChild(m_MecanumDrive, m_leftFrontMotor);
    SendableRegistry.addChild(m_MecanumDrive, m_leftBackMotor);
    SendableRegistry.addChild(m_MecanumDrive, m_rightFrontMotor);
    SendableRegistry.addChild(m_MecanumDrive, m_rightBackMotor);




     m_leftBackMotor.setInverted(true);
     m_leftFrontMotor.setInverted(true);


 


    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    //set 1 variable = to left motors, and 1 variable to the left motors

   
  }
 
  @Override
  public void teleopInit() {
    m_elevatorEncoder.reset();
    m_clawEncoder.reset();
    lastTimestamp_elevator = m_Timer.getFPGATimestamp();
    errorSum_elevator = 0;
    lastError_elevator = 0;
    ElevatorPosition = 0;

  }
  @Override
  public void teleopPeriodic() {
    m_MecanumDrive.driveCartesian(-m_XboxController.getLeftY(), -m_XboxController.getLeftX(), m_XboxController.getRightX());

    // if (m_XboxController.getAButton() == true){
    //   m_elevator.set(-0.25);
    // } else {
    //   m_elevator.set(0);
    // }
   
   
   
    if (m_XboxController.getAButtonPressed() == true) {
      ElevatorPosition = 1.2083333; // 1.2083333
    } else if (m_XboxController.getBButtonPressed() == true){
      ElevatorPosition = 0;
    } else {
      //gfdhge
    }

    sensorPosition_elevator = -m_elevatorEncoder.getDistance() / 4020;
    error_elevator = ElevatorPosition - sensorPosition_elevator;
    dT_elevator = m_Timer.getFPGATimestamp() - lastTimestamp_elevator;

    if (Math.abs(error_elevator) < iLimit_elevator){
      errorSum_elevator += (error_elevator * dT_elevator);
    }

    errorRate_elevator = (error_elevator - lastError_elevator) / dT_elevator;

    outputSpeed_elevator = kP_elevator * error_elevator +
      kI_elevator * errorSum_elevator +
      kD_elevator * errorRate_elevator;

    m_elevator.set(outputSpeed_elevator);

    lastTimestamp_elevator = m_Timer.getFPGATimestamp();
    lastError_elevator = error_elevator;






    sensorPosition_claw = m_clawEncoder.getDistance() / 1; // find variable
    error_claw = ClawPosition - sensorPosition_claw;

    outputSpeed_claw = kP_claw * error_claw;

    //m_claw.set(outputSpeed_claw);



    if (m_XboxController.getXButton() == true){
      System.out.println(m_elevatorEncoder.getDistance() + " - ");
    }

   
  }  
}
