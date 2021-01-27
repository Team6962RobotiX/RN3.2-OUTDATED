package frc.robot;

//Utilities
  import edu.wpi.first.wpilibj.TimedRobot;
  import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
  import java.util.*;

//Joystick Imports
  import edu.wpi.first.wpilibj.Joystick;

//Encoders
  import edu.wpi.first.wpilibj.Encoder;

//Limit Switches
  import edu.wpi.first.wpilibj.DigitalInput;

//Motor Imports
  import edu.wpi.first.wpilibj.Spark;
  import com.ctre.phoenix.motorcontrol.can.VictorSPX;
  import edu.wpi.first.wpilibj.VictorSP;
  import com.ctre.phoenix.motorcontrol.ControlMode;

//Drive Train Imports
  import edu.wpi.first.wpilibj.drive.DifferentialDrive;

// Camera
  import edu.wpi.cscore.CvSink;
  import edu.wpi.cscore.CvSource;
  import edu.wpi.cscore.UsbCamera;
  import edu.wpi.first.cameraserver.CameraServer;
  import org.opencv.core.Core;
  import org.opencv.core.Mat;
  import org.opencv.imgproc.Imgproc;
  import frc.robot.FindBall;
  import frc.robot.FindTarget;

public class Robot extends TimedRobot {

  //Joysticks
    Joystick joystick0;
    Joystick joystick1;

  //Drivetrain
    DifferentialDrive myDrive;

  //Motors

    //Sparks
      Spark lbank;
      Spark rbank;
      Spark transfer;
      Spark outtake;

    //Victors
      VictorSP drawer;
      VictorSPX intake;

  //Encoders
    Encoder encoder1;
    Encoder encoder2;

  //Limit Switches
    DigitalInput drawerIn;
    DigitalInput drawerOut;
    DigitalInput startBelt;
    DigitalInput stopBelt;

  //Gun Values
    long startDelay = 0; //EDITABLE VALUE!!!
    boolean pullIn = true;
    boolean runIntake = true;
  
  //Camera
    UsbCamera camera;
    double[] targetAngleValue = new double[ 1 ];
    boolean[] setTargetAngleValue = new boolean[ 1 ];
    double[] ballAngleValue = new double[ 1 ]; 
    boolean[] setBallAngleValue = new boolean[ 1 ];
    public static final int WINDOW_WIDTH = 640;
    public static final int WINDOW_HEIGHT = 480;
    Mat cameraMatrix;
    Mat distCoeffs;
    CvSink cvSink;
    Mat source;

  @Override
  public void robotInit() {

    //Joystick
      joystick0 = new Joystick(0);
      joystick1 = new Joystick(1);

    //Sparks
      rbank = new Spark(0);
      lbank = new Spark(1);
      transfer = new Spark(2);
      outtake = new Spark(4);

    //Victors
      drawer = new VictorSP(9);
      intake = new VictorSPX(10);

    //Drive Train
      myDrive = new DifferentialDrive(lbank, rbank);

    //Encoders
      encoder1 = new Encoder( 0, 1 ); //Left Encoder
      encoder2 = new Encoder( 2, 3 ); //Right Encoder

    //Limit Switches
      drawerIn = new DigitalInput( 9 );
      drawerOut = new DigitalInput( 8 );
      startBelt = new DigitalInput( 7 );
      stopBelt = new DigitalInput( 6 );

    //Camera
      ballAngleValue[0] = -1;

      System.loadLibrary( Core.NATIVE_LIBRARY_NAME );
      cameraMatrix = new Mat();
      distCoeffs = new Mat();

      new Thread( () -> {
    
        FindTarget.setup();
        FindBall.readCalibrationData( "calib-logitech.mov-720-30-calib.txt", cameraMatrix, distCoeffs );
  
        UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
        camera.setResolution( WINDOW_WIDTH, WINDOW_HEIGHT );
        CvSink cvSink = CameraServer.getInstance().getVideo();
        CvSource cvSource = CameraServer.getInstance().putVideo("test", WINDOW_WIDTH, WINDOW_HEIGHT);
        Mat output = new Mat();
        //CvSource outputStream = CameraServer.getInstance().putVideo( "Blur", 640, 480 );
  
        Mat source = new Mat();
        //Mat output = new Mat();
        int i = 0;
  
        while( !Thread.interrupted() ) {
          
          if ( cvSink.grabFrame( source ) == 0 ) {
            SmartDashboard.putString("Status", cvSink.getError());
          }
          else{
            SmartDashboard.putString("Status", "success" + Integer.toString(i));
            i++;
  
          }
          output = FindBall.displayContours(source, WINDOW_WIDTH, WINDOW_HEIGHT, cameraMatrix, distCoeffs);
          if(output != null && !output.empty())cvSource.putFrame(output);
          ballAngleValue[ 0 ] = FindBall.getBallValue( source, WINDOW_WIDTH, WINDOW_HEIGHT, cameraMatrix, distCoeffs );
          setBallAngleValue [ 0 ] = true;
          SmartDashboard.putNumber("ballAngleValue", ballAngleValue[0]);
          System.gc();
  
        }
        
      } ).start();

  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Left Encoder", encoder1.getDistance());
    SmartDashboard.putNumber("Right Encoder", encoder2.getDistance());
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopPeriodic() {

    //Turning speed limit
      double limitTurnSpeed = 0.75; //EDITABLE VALUE

    //Default manual Drive Values
      double joystickLValue = ( -joystick0.getRawAxis( 1 ) + ( joystick0.getRawAxis( 2 ) * limitTurnSpeed ) );
      double joystickRValue = ( -joystick0.getRawAxis( 1 ) - ( joystick0.getRawAxis( 2 ) * limitTurnSpeed ) );




    //ADDITIONAL DRIVE CODE HERE




    //Gun
      //Transfer
        if(joystick0.getRawButton(2)){
          runIntake = false;
        }else if(joystick0.getRawButton(3)){
          runIntake = true;
        }

        SmartDashboard.putBoolean( "canTransfer?", startBelt.get() );
        if( !startBelt.get() ) {
        
          transfer.set( -1 );
          startDelay = System.currentTimeMillis();
        
        } else if( !stopBelt.get() ) transfer.set( 0 );
        if(runIntake)intake.set(ControlMode.PercentOutput, -0.75);
          else intake.set(ControlMode.PercentOutput, 0);

        if( !( transfer.get() == 0 ) ) intake.set( ControlMode.PercentOutput, -0.5 );
        if(runIntake)intake.set(ControlMode.PercentOutput, -0.75);
          else intake.set(ControlMode.PercentOutput, 0);

      //Outtake
        if( joystick0.getRawButton( 1 ) ) {
        
          intake.set( ControlMode.PercentOutput, 0 );
          transfer.set( -1 );

        } else {
          if(runIntake)intake.set(ControlMode.PercentOutput, -0.75);
          else intake.set(ControlMode.PercentOutput, 0);
          transfer.set(0);
        }
        outtake.set( -( joystick0.getRawAxis( 3 ) - 1 ) / 2 );

      //Drawer +ve = in && -ve = out
      if( joystick0.getRawButton( 7 ) ) pullIn = false;
      if( joystick0.getRawButton( 8 ) ) pullIn = true;
      if( pullIn && drawerIn.get() ) drawer.set( 0.7 );
      else if( !pullIn && drawerOut.get() ) drawer.set( -0.7 );
      else drawer.set( 0 );

    //Forgive a slight turn
      if( joystickLValue - joystickRValue < 0.2 && joystickLValue - joystickRValue > -0.2 ){
        joystickLValue = joystickRValue;
      }

    //Actual Drive code
      myDrive.tankDrive(joystickLValue,joystickRValue);
  }

  @Override
  public void testPeriodic() {
  }
}
