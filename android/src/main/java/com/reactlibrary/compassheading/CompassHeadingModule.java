package com.reactlibrary.compassheading;

import com.facebook.react.bridge.ReactApplicationContext;
import com.facebook.react.bridge.ReactContextBaseJavaModule;
import com.facebook.react.bridge.ReactMethod;
import com.facebook.react.bridge.Promise;
import com.facebook.react.bridge.Arguments;
import com.facebook.react.bridge.WritableMap;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.view.Display;
import android.view.WindowManager;
import android.view.Surface;

import android.content.Context;

import com.facebook.react.modules.core.DeviceEventManagerModule;


public class CompassHeadingModule extends ReactContextBaseJavaModule implements SensorEventListener {

    private static Context mApplicationContext;
    private SensorManager sensorManager;

    private int mAzimuth = 0; // degree
    private float mFilter = 1;

    private final float[] mGravity = new float[3];
    private final float[] mGeomagnetic = new float[3];

    private final float[] R = new float[9];
    private final float[] I = new float[9];

    public CompassHeadingModule(ReactApplicationContext reactContext) {
        super(reactContext);

        mApplicationContext = reactContext.getApplicationContext();
    }

    @Override
    public String getName() {
        return "CompassHeading";
    }

    @ReactMethod
    public void start(int filter, Promise promise) {

        try{
            sensorManager = (SensorManager) mApplicationContext.getSystemService(Context.SENSOR_SERVICE);

            assert sensorManager != null;
            Sensor gsensor = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
            Sensor msensor = sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
            Sensor rsensor = sensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR);

            boolean rotationVectorExists = sensorManager.registerListener(this,
                rsensor,
                SensorManager.SENSOR_DELAY_GAME);

            if(!rotationVectorExists){
                sensorManager.registerListener(this, gsensor, SensorManager.SENSOR_DELAY_GAME);
                sensorManager.registerListener(this, msensor, SensorManager.SENSOR_DELAY_GAME);
            };

            mFilter = filter;
            promise.resolve(true);
        }
        catch(Exception e){
            promise.reject("failed_start", e.getMessage());
        }
    }

    @ReactMethod
    public void stop() {
        if (sensorManager != null) {
            sensorManager.unregisterListener(this);
        }
    }

    @ReactMethod
    public void hasCompass(Promise promise) {

        try{
            SensorManager manager = (SensorManager) mApplicationContext.getSystemService(Context.SENSOR_SERVICE);

            assert manager != null;
            boolean res = manager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR) != null ||
                (manager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER) != null &&
                manager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD) != null);

            promise.resolve(res);
        }
        catch(Exception e){
            promise.resolve(false);
        }
    }

    @Override
    public void onSensorChanged(SensorEvent event) {
        float[] mMatrixR = new float[9];
        float[] outRotationMatrix;
        float[] mMatrixValues = new float[3];
        final float alpha = 0.97f;

        synchronized (this) {
            if (event.sensor.getType() == Sensor.TYPE_ROTATION_VECTOR) {
                SensorManager.getRotationMatrixFromVector(mMatrixR, event.values);
                SensorManager.getOrientation(mMatrixR, mMatrixValues);

                int roll = (int) Math.toDegrees(mMatrixValues[2]);
                int pitch = (int) Math.toDegrees(mMatrixValues[1]);

                outRotationMatrix = remapCoordinatesSystem(mMatrixR, roll, pitch);
                SensorManager.getOrientation(outRotationMatrix, mMatrixValues);

                int newAzimuth = (int) Math.toDegrees(mMatrixValues[0]);

                emitResponse((newAzimuth + 360) % 360, event.accuracy);
            } else {
                if (event.sensor.getType() == Sensor.TYPE_ACCELEROMETER) {

                    mGravity[0] = alpha * mGravity[0] + (1 - alpha)
                        * event.values[0];
                    mGravity[1] = alpha * mGravity[1] + (1 - alpha)
                        * event.values[1];
                    mGravity[2] = alpha * mGravity[2] + (1 - alpha)
                        * event.values[2];
                }

                if (event.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD) {

                    mGeomagnetic[0] = alpha * mGeomagnetic[0] + (1 - alpha)
                        * event.values[0];
                    mGeomagnetic[1] = alpha * mGeomagnetic[1] + (1 - alpha)
                        * event.values[1];
                    mGeomagnetic[2] = alpha * mGeomagnetic[2] + (1 - alpha)
                        * event.values[2];

                }

                boolean success = SensorManager.getRotationMatrix(R, I, mGravity, mGeomagnetic);

                if (success) {

                    float[] orientation = new float[3];
                    SensorManager.getOrientation(R, orientation);

                    int newAzimuth = (int) Math.toDegrees(orientation[0]);
                    newAzimuth = displayRotation((newAzimuth + 360) % 360);
                    emitResponse(newAzimuth, event.accuracy);
                }
            }
        }
    }

    public float[] remapCoordinatesSystem (float[] rotationMatrix, int roll, int pitch) {
        Display disp = (((WindowManager) mApplicationContext.getSystemService(Context.WINDOW_SERVICE))).getDefaultDisplay();
        float[] outRotationMatrix = new float[9];
        int rotation = disp.getRotation();


        switch (rotation) {
            case Surface.ROTATION_0:
                if(pitch < -10){
                    SensorManager.remapCoordinateSystem(rotationMatrix, SensorManager.AXIS_X, SensorManager.AXIS_Z, outRotationMatrix);
                } else {
                    SensorManager.remapCoordinateSystem(rotationMatrix, SensorManager.AXIS_X, SensorManager.AXIS_Y, outRotationMatrix);
                }
                break;
            case Surface.ROTATION_90:
                if(roll < -40){
                    SensorManager.remapCoordinateSystem(rotationMatrix, SensorManager.AXIS_X, SensorManager.AXIS_Z, outRotationMatrix);
                } else {
                    SensorManager.remapCoordinateSystem(rotationMatrix, SensorManager.AXIS_Y, SensorManager.AXIS_MINUS_X, outRotationMatrix);
                }
                break;
            case Surface.ROTATION_180:
                if(pitch > 10){
                    SensorManager.remapCoordinateSystem(rotationMatrix, SensorManager.AXIS_X, SensorManager.AXIS_Z, outRotationMatrix);
                } else {
                    SensorManager.remapCoordinateSystem(rotationMatrix, SensorManager.AXIS_MINUS_X, SensorManager.AXIS_MINUS_Y, outRotationMatrix);
                }
                break;
            case Surface.ROTATION_270:
                if(roll > 40){
                    SensorManager.remapCoordinateSystem(rotationMatrix, SensorManager.AXIS_X, SensorManager.AXIS_Z, outRotationMatrix);
                } else {
                    SensorManager.remapCoordinateSystem(rotationMatrix, SensorManager.AXIS_MINUS_Y, SensorManager.AXIS_X, outRotationMatrix);
                }
                break;
        }


        return outRotationMatrix;
    }

    public int displayRotation (int azimuth) {
        Display disp = (((WindowManager) mApplicationContext.getSystemService(Context.WINDOW_SERVICE))).getDefaultDisplay();

        int newAzimuth = azimuth;

        if(disp != null){
            int rotation = disp.getRotation();

            if(rotation == Surface.ROTATION_90){
                newAzimuth = (azimuth + 90) % 360;
            }
            else if(rotation == Surface.ROTATION_270){
                newAzimuth = (azimuth + 270) % 360;
            }
            else if(rotation == Surface.ROTATION_180){
                newAzimuth = (azimuth + 180) % 360;
            }
        }

        return newAzimuth;
    }

    public void emitResponse (int newAzimuth, int accuracy) {
        if (Math.abs(mAzimuth - newAzimuth) > mFilter) {

            mAzimuth = newAzimuth;

            WritableMap params = Arguments.createMap();
            params.putDouble("heading", mAzimuth);
            params.putDouble("accuracy", accuracy);

            getReactApplicationContext()
                .getJSModule(DeviceEventManagerModule.RCTDeviceEventEmitter.class)
                .emit("HeadingUpdated", params);
        }
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {
    }
}
