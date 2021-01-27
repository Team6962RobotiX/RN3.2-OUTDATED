package frc.robot;
import edu.wpi.first.networktables.*;

public class FindTarget {

	public static NetworkTable table;
	public static double angleFrontPortValue = 0d;
	public static double angleBackPortValue = 0d;
	public static boolean trackingSuccess = false;
	public static boolean setTargetValue;
	public static double targetValue;
	public static final int TEAM_NUMBER = 6962;

	public static void setup() {
		// https://docs.wpilib.org/en/latest/docs/software/networktables/listening-for-change.html
		FindTarget.table = NetworkTableInstance.getDefault().getTable("SmartDashboard");
		FindTarget.setTargetValue = false;
		FindTarget.targetValue = 0d;

		FindTarget.table.addEntryListener("AngleFrontPort", (table, key, entry, value, flags) -> {
			FindTarget.angleFrontPortValue = Double.parseDouble(value.getValue().toString());
		}, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

		FindTarget.table.addEntryListener("AngleBackPort", (table, key, entry, value, flags) -> {
			FindTarget.angleBackPortValue = Double.parseDouble(value.getValue().toString());
		}, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

		FindTarget.table.addEntryListener("TrackingSuccess", (table, key, entry, value, flags) -> {
			FindTarget.trackingSuccess = value.getValue().toString().equals("true");
		}, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
	}

	public static double getAngleFrontPortValue() {
		if(FindTarget.trackingSuccess) { return FindTarget.angleFrontPortValue; }
		return 0d;
	}

	public static double getAngleBackPortValue() {
		if(FindTarget.trackingSuccess) { return FindTarget.angleBackPortValue; }
		return 0d;
	}
}