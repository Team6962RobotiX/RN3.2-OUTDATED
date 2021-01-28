package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.opencv.core.*;
import org.opencv.core.Mat;

import java.util.*;

public class Robot extends TimedRobot {

    // Joysticks
    Joystick joystick0;
    Joystick joystick1;

    // Drivetrain
    DifferentialDrive myDrive;

    // Motors

    // Sparks
    Spark lbank;
    Spark rbank;
    Spark transfer;
    Spark outtake;

    // Victors
    VictorSP drawer;
    VictorSPX intake;

    // Encoders
    Encoder encoder1;
    Encoder encoder2;

    // Limit Switches
    DigitalInput drawerIn;
    DigitalInput drawerOut;
    DigitalInput startBelt;
    DigitalInput stopBelt;

    // Gun Values
    long startDelay = 0; // EDITABLE VALUE!!!
    boolean pullIn = true;
    boolean runIntake = true;

    // Camera
    UsbCamera camera;
    double[] targetAngleValue = new double[1];
    boolean[] setTargetAngleValue = new boolean[1];
    double[] ballAngleValue = new double[1];
    boolean[] setBallAngleValue = new boolean[1];
    public static final int WINDOW_WIDTH = 640;
    public static final int WINDOW_HEIGHT = 480;
    Mat cameraMatrix;
    Mat distCoeffs;
    CvSink cvSink;
    Mat source;

    // Autonomous
    ArrayList<double[]> path = new ArrayList<double[]>();
    double correctionFactor = 0.01;
    int clock = 100;
    long start;

    @Override
    public void robotInit() {

        // Joystick
        joystick0 = new Joystick(0);
        joystick1 = new Joystick(1);

        // Sparks
        rbank = new Spark(0);
        lbank = new Spark(1);
        transfer = new Spark(2);
        outtake = new Spark(4);

        // Victors
        drawer = new VictorSP(9);
        intake = new VictorSPX(10);

        // Drive Train
        myDrive = new DifferentialDrive(lbank, rbank);

        // Encoders
        encoder1 = new Encoder(0, 1); // Left Encoder
        encoder2 = new Encoder(2, 3); // Right Encoder

        // Limit Switches
        drawerIn = new DigitalInput(9);
        drawerOut = new DigitalInput(8);
        startBelt = new DigitalInput(7);
        stopBelt = new DigitalInput(6);

        // Camera
        ballAngleValue[0] = -1;

        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
        cameraMatrix = new Mat();
        distCoeffs = new Mat();

        new Thread(
                        () -> {
                            FindTarget.setup();
                            FindBall.readCalibrationData(
                                    "3 3\n"
                                            + "9.292197792782764054e+02 0.000000000000000000e+00"
                                            + " 6.771372380756025677e+02\n"
                                            + "0.000000000000000000e+00 9.405077247570802683e+02"
                                            + " 3.447556745828786120e+02\n"
                                            + "0.000000000000000000e+00 0.000000000000000000e+00"
                                            + " 1.000000000000000000e+00\n"
                                            + "1 5\n"
                                            + "6.677862607245312054e-02 -3.771175521797300728e-02"
                                            + " -4.856974855773007159e-03 2.624119205577779904e-03"
                                            + " -1.404855226310152971e-01",
                                    cameraMatrix,
                                    distCoeffs);

                            UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
                            camera.setResolution(WINDOW_WIDTH, WINDOW_HEIGHT);
                            CvSink cvSink = CameraServer.getInstance().getVideo();
                            CvSource cvSource =
                                    CameraServer.getInstance()
                                            .putVideo("test", WINDOW_WIDTH, WINDOW_HEIGHT);
                            Mat output = new Mat();
                            // CvSource outputStream = CameraServer.getInstance().putVideo( "Blur",
                            // 640, 480 );

                            Mat source = new Mat();
                            // Mat output = new Mat();
                            int i = 0;
                            boolean flag = true;

                            while (!Thread.interrupted()) {

                                if (cvSink.grabFrame(source) == 0) {
                                    SmartDashboard.putString("Status", cvSink.getError());
                                    flag = false;
                                } else {
                                    SmartDashboard.putString(
                                            "Status", "success" + Integer.toString(i));
                                    i++;
                                    flag = true;
                                }
                                output =
                                        FindBall.displayContours(
                                                source,
                                                WINDOW_WIDTH,
                                                WINDOW_HEIGHT,
                                                cameraMatrix,
                                                distCoeffs);
                                if (output != null && !output.empty()) {
                                    cvSource.putFrame(output);
                                } else {
                                    if (flag) {
                                        cvSource.putFrame(source);
                                    }
                                }
                                ballAngleValue[0] =
                                        FindBall.getBallValue(
                                                source,
                                                WINDOW_WIDTH,
                                                WINDOW_HEIGHT,
                                                cameraMatrix,
                                                distCoeffs);
                                setBallAngleValue[0] = true;
                                SmartDashboard.putNumber("ballAngleValue", ballAngleValue[0]);
                                System.gc();
                            }
                        })
                .start();
    }

    @Override
    public void robotPeriodic() {
        SmartDashboard.putNumber("Left Encoder", encoder1.getDistance());
        SmartDashboard.putNumber("Right Encoder", encoder2.getDistance());
    }

    @Override
    public void autonomousInit() {
        start = System.currentTimeMillis();
        encoder1.reset();
        encoder2.reset();
        // path = new ArrayList<int[]>();

        // Scanner sc;
        // try {
        //     sc = new Scanner(new File("[FILEPATH]"));
        // } catch (Exception e) {
        //     sc = new Scanner("0,0\n0,5\n10,20\n20,25\n30,30");
        // }
        // sc.useDelimiter(",");
        // while (sc.hasNext()) {
        //     path.add(new int[] {sc.nextInt(), sc.nextInt()});
        // }
        // sc.close();
    }

    @Override
    public void autonomousPeriodic() {
        long now = (System.currentTimeMillis() - start);

        int step = (int) Math.floor(now / clock); // index of path we're on or going through
        double substep = ((double) (now % clock)) / clock; // % of the way through current path step

        if (step < path.size() - 1) {
            double[] current = path.get(step);
            double[] next = path.get(step + 1);

            double targetLeft = (next[0] - current[0]) * substep;
            double realLeft = encoder1.getDistance() - current[0];
            if (realLeft == 0) realLeft = 0.01;
            double correctionLeft = targetLeft / realLeft;
            correctionLeft = ((correctionLeft - 1) * correctionFactor) + 1;

            double targetRight = (next[1] - current[1]) * substep;
            double realRight = encoder2.getDistance() - current[1];
            if (realRight == 0) realRight = 0.01;
            double correctionRight = targetRight / realRight;
            correctionRight = ((correctionRight - 1) * correctionFactor) + 1;

            System.out.println(
                    "Target left: "
                            + Double.toString(targetLeft)
                            + ", Defecit: "
                            + Double.toString(realLeft)
                            + ", Raw correction: "
                            + Double.toString(correctionLeft));
            // myDrive.tankDrive(correctionLeft, correctionRight);
            myDrive.tankDrive(0, 0);
        } else if (step < path.size()) {
            double[] target = path.get(step);

            double targetLeft = target[0] * substep;
            double realLeft = encoder1.getDistance();
            if (realLeft == 0) realLeft = 0.01;
            double correctionLeft = targetLeft / realLeft;
            correctionLeft = ((correctionLeft - 1) * correctionFactor) + 1;

            double targetRight = target[1] * substep;
            double realRight = encoder2.getDistance();
            if (realRight == 0) realRight = 0.01;
            double correctionRight = targetRight / realRight;
            correctionRight = ((correctionRight - 1) * correctionFactor) + 1;

            System.out.println(
                    "Target left: "
                            + Double.toString(targetLeft)
                            + ", Defecit: "
                            + Double.toString(realLeft)
                            + ", Raw correction: "
                            + Double.toString(correctionLeft));
            // myDrive.tankDrive(correctionLeft, correctionRight);
            myDrive.tankDrive(0, 0);
        } else {
            myDrive.tankDrive(0, 0);
        }
    }

    @Override
    public void teleopInit() {
        start = System.currentTimeMillis();
        encoder1.reset();
        encoder2.reset();
    }

    @Override
    public void teleopPeriodic() {

        // Turning speed limit
        double limitTurnSpeed = 0.75; // EDITABLE VALUE

        // Default manual Drive Values
        double joystickLValue =
                (-joystick0.getRawAxis(1) + (joystick0.getRawAxis(2) * limitTurnSpeed));
        double joystickRValue =
                (-joystick0.getRawAxis(1) - (joystick0.getRawAxis(2) * limitTurnSpeed));

        // ADDITIONAL DRIVE CODE HERE

        // Gun
        // Transfer
        if (joystick0.getRawButton(2)) {
            runIntake = false;
        } else if (joystick0.getRawButton(3)) {
            runIntake = true;
        }

        SmartDashboard.putBoolean("canTransfer?", startBelt.get());
        if (!startBelt.get()) {

            transfer.set(-1);
            startDelay = System.currentTimeMillis();

        } else if (!stopBelt.get()) transfer.set(0);
        if (runIntake) intake.set(ControlMode.PercentOutput, -0.75);
        else intake.set(ControlMode.PercentOutput, 0);

        if (!(transfer.get() == 0)) intake.set(ControlMode.PercentOutput, -0.5);
        if (runIntake) intake.set(ControlMode.PercentOutput, -0.75);
        else intake.set(ControlMode.PercentOutput, 0);

        // Outtake
        if (joystick0.getRawButton(1)) {

            intake.set(ControlMode.PercentOutput, 0);
            transfer.set(-1);

        } else {
            if (runIntake) intake.set(ControlMode.PercentOutput, -0.75);
            else intake.set(ControlMode.PercentOutput, 0);
            transfer.set(0);
        }
        outtake.set(-(joystick0.getRawAxis(3) - 1) / 2);

        // Drawer +ve = in && -ve = out
        if (joystick0.getRawButton(7)) pullIn = false;
        if (joystick0.getRawButton(8)) pullIn = true;
        if (pullIn && drawerIn.get()) drawer.set(0.7);
        else if (!pullIn && drawerOut.get()) drawer.set(-0.7);
        else drawer.set(0);

        // Forgive a slight turn
        if (joystickLValue - joystickRValue < 0.2 && joystickLValue - joystickRValue > -0.2) {
            joystickLValue = joystickRValue;
        }

        // Actual Drive code
        myDrive.tankDrive(joystickLValue, joystickRValue);

        // Pathing Stuff
        long now = (System.currentTimeMillis() - start);

        int step = (int) (now / clock); // index of path we're on or going through
        double substep = ((double) (now % clock)) / clock; // % of the way through current path step

        if (path.size() <= step) {
            if (step == 0) path.add(new double[] {0.0, 0.0});
            else {
                // estimate position at time of step
                double leftDistance =
                        ((encoder1.getDistance() - path.get(step - 1)[0]) * substep)
                                + path.get(step - 1)[0];
                double rightDistance =
                        ((encoder2.getDistance() - path.get(step - 1)[1]) * substep)
                                + path.get(step - 1)[1];

                path.add(new double[] {leftDistance, rightDistance});
            }
        }
    }

    @Override
    public void testPeriodic() {}
}
