package frc.robot.util;

import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;

import org.json.JSONArray;
import org.json.JSONObject;
import org.json.JSONTokener;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import me.nabdev.oxconfig.ConfigurableParameter;

public class AutoAimDataManager {
    public static final ConfigurableParameter<Boolean> calibrationMode = new ConfigurableParameter<>(false,
            "Shooter Calibration Mode");
    public static final ConfigurableParameter<Boolean> mzCalibrationMode = new ConfigurableParameter<>(false,
            "Shooter Midzone Calibration Mode");

    private static JSONObject azConfig;
    private JSONObject mzConfig;
    private TreeMapRecord azTreeMapRecord;
    private TreeMapRecord mzTreeMapRecord;

    public record TreeMapRecord(InterpolatingDoubleTreeMap flywheelSpeedMap,
            InterpolatingDoubleTreeMap hoodPositionMap,
            InterpolatingDoubleTreeMap timeOfFlightMap, double minDistance, double maxDistance) {
    }

    public AutoAimDataManager() throws FileNotFoundException {
        FileInputStream input = new FileInputStream(Filesystem.getDeployDirectory() + "/azaim.json");
        JSONTokener tokener = new JSONTokener(input);
        azConfig = new JSONObject(tokener);

        input = new FileInputStream(Filesystem.getDeployDirectory() + "/mzaim.json");
        tokener = new JSONTokener(input);
        mzConfig = new JSONObject(tokener);

        updateFromJson(false);
        updateFromJson(true);
    }

    private void updateFromJson(boolean mz) {
        JSONObject config = mz ? mzConfig : azConfig;
        JSONArray shots = config.getJSONArray("shots");
        double minDistance = Double.POSITIVE_INFINITY;
        double maxDistance = Double.NEGATIVE_INFINITY;

        InterpolatingDoubleTreeMap flywheelSpeedMap = new InterpolatingDoubleTreeMap();
        InterpolatingDoubleTreeMap hoodPositionMap = new InterpolatingDoubleTreeMap();
        InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();
        for (int i = 0; i < shots.length(); i++) {
            JSONObject shot = shots.getJSONObject(i);
            double distance = shot.getDouble("distance");
            double hoodPosition = shot.getDouble("hoodPosition");
            double shooterSpeed = shot.getDouble("shooterSpeed");
            double shotTime = shot.getDouble("shotTime");
            if (distance < minDistance) {
                minDistance = distance;
            }
            if (distance > maxDistance) {
                maxDistance = distance;
            }
            flywheelSpeedMap.put(distance, shooterSpeed);
            hoodPositionMap.put(distance, hoodPosition);
            timeOfFlightMap.put(distance, shotTime);
        }
        if (shots.length() == 0) {
            flywheelSpeedMap.put(0.0, 0.0);
            hoodPositionMap.put(0.0, 0.0);
            timeOfFlightMap.put(0.0, 0.0);
            minDistance = 0.0;
            maxDistance = 0.0;
        }
        if (mz) {
            mzTreeMapRecord = new TreeMapRecord(flywheelSpeedMap, hoodPositionMap, timeOfFlightMap, minDistance,
                    maxDistance);
        } else {
            azTreeMapRecord = new TreeMapRecord(flywheelSpeedMap, hoodPositionMap, timeOfFlightMap, minDistance,
                    maxDistance);
        }

        try (FileWriter file = new FileWriter(Filesystem.getDeployDirectory() + (mz ? "/mzaim.json" : "/azaim.json"))) {
            file.write(config.toString());
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void addShot(double distance, double hoodPosition, double shooterSpeed) {
        JSONObject recordJson = new JSONObject().put("distance", distance).put("hoodPosition", hoodPosition)
                .put("shooterSpeed", shooterSpeed).put("shotTime", 0)
                .put("timestamp", System.currentTimeMillis());
        if (mzCalibrationMode.get()) {
            mzConfig.getJSONArray("shots")
                    .put(recordJson);
        } else if (calibrationMode.get()) {
            azConfig.getJSONArray("shots")
                    .put(recordJson);
        }
        updateFromJson(mzCalibrationMode.get());
    }

    public void periodic() {
        if (!calibrationMode.get()) {
            return;
        }
        SmartDashboard.putString("AzAutoAim", azConfig.toString());
        SmartDashboard.putString("MzAutoAim", mzConfig.toString());
        String azSet = SmartDashboard.getString("AzAutoAimSet", "");
        if (!azSet.isEmpty()) {
            JSONTokener tokener = new JSONTokener(azSet);
            azConfig = new JSONObject(tokener);
            updateFromJson(false);
        }
        String mzSet = SmartDashboard.getString("MzAutoAimSet", "");
        if (!mzSet.isEmpty()) {
            JSONTokener tokener = new JSONTokener(mzSet);
            mzConfig = new JSONObject(tokener);
            updateFromJson(true);
        }
    }

    public InterpolatingDoubleTreeMap getShotFlywheelSpeedMap(boolean mz) {
        return mz ? mzTreeMapRecord.flywheelSpeedMap() : azTreeMapRecord.flywheelSpeedMap();
    }

    public InterpolatingDoubleTreeMap getShotHoodPositionMap(boolean mz) {
        return mz ? mzTreeMapRecord.hoodPositionMap() : azTreeMapRecord.hoodPositionMap();
    }

    public InterpolatingDoubleTreeMap getTimeOfFlightMap(boolean mz) {
        return mz ? mzTreeMapRecord.timeOfFlightMap() : azTreeMapRecord.timeOfFlightMap();
    }

    public double getMinDistance(boolean mz) {
        return mz ? mzTreeMapRecord.minDistance() : azTreeMapRecord.minDistance();
    }

    public double getMaxDistance(boolean mz) {
        return mz ? mzTreeMapRecord.maxDistance() : azTreeMapRecord.maxDistance();
    }
}
