package com.kebbi.myapplication12.robot;

import android.content.Context;
import android.os.Handler;
import android.util.Log;

import androidx.annotation.Nullable;

import com.nuwarobotics.service.agent.NuwaRobotAPI;
import com.nuwarobotics.service.agent.RobotEventListener;
import com.nuwarobotics.service.agent.VoiceEventListener;
import com.nuwarobotics.service.agent.VoiceEventListener.HotwordState;
import com.nuwarobotics.service.agent.VoiceEventListener.HotwordType;
import com.nuwarobotics.service.agent.VoiceEventListener.ListenType;
import com.nuwarobotics.service.agent.VoiceEventListener.ResultType;
import com.nuwarobotics.service.agent.VoiceEventListener.SpeakState;
import com.nuwarobotics.service.agent.VoiceEventListener.SpeakType;
import com.nuwarobotics.service.agent.VoiceEventListener.SpeechState;

import java.util.Locale;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

/**
 * 封裝 NuwaRobotAPI 控制、里程計與睡眠/喚醒等功能，供其他模組呼叫。
 */
public class RobotMotionController {

    private static final String TAG = "RobotMotion";

    private static final int GRID_SIZE = 320;
    private static final float ROI_X_MIN = -2f;
    private static final float ROI_X_MAX = 2f;
    private static final float ROI_Z_MIN = 0f;
    private static final float ROI_Z_MAX = 4f;
    private static final float CELL_M = (ROI_X_MAX - ROI_X_MIN) / GRID_SIZE;

    public static final float DEFAULT_MOVE_SPEED = 0.12f;     // m/s
    public static final float DEFAULT_TURN_SPEED = 30.053f;   // deg/s (右轉為正)

    public interface MotionListener {
        void onOdometryUpdate(int row, int col, float xMeters, float zMeters, float headingDeg);

        void onRobotEvent(String message);
    }

    private final Context appContext;
    private final Handler uiHandler;
    private final MotionListener motionListener;

    private NuwaRobotAPI robotApi;

    private final Object robotLock = new Object();
    private float posX_m = 0f;
    private float posZ_m = 0f;
    // 讓開機面朝「前方」對應世界座標 +Y，預設 yaw=90 度（與 Python 端初始朝向一致）
    private float headingDeg = 90f;
    private float cmdMoveSpeed_mps = 0f;
    private float cmdTurnSpeed_dps = 0f;

    private ScheduledExecutorService posTimer;
    private volatile long lastOdoNanos = 0L;

    public RobotMotionController(Context context, Handler uiHandler, MotionListener listener) {
        this.appContext = context.getApplicationContext();
        this.uiHandler = uiHandler;
        this.motionListener = listener;
    }

    public void start() {
        robotApi = initNuwaApiCompat();
        if (robotApi != null) {
            registerRobotListenerSafe();
            registerVoiceListenerSafe();
        }
        startPositionTimer();
    }

    public void stop() {
        stopPositionTimer();
        if (robotApi != null) {
            unregisterRobotListenerSafe();
            unregisterVoiceListenerSafe();
            callRobot("release", new Class[]{}, new Object[]{});
            robotApi = null;
        }
        stopMotion();
    }

    /* ------------ 基礎移動控制 ------------- */
    public void moveForward() {
        moveContinuous(DEFAULT_MOVE_SPEED);
    }

    public void moveBackward() {
        moveContinuous(-DEFAULT_MOVE_SPEED);
    }

    public void turnLeft() {
        turnContinuous(-DEFAULT_TURN_SPEED);
    }

    public void turnRight() {
        turnContinuous(DEFAULT_TURN_SPEED);
    }

    public void stopMotion() {
        synchronized (robotLock) {
            cmdMoveSpeed_mps = 0f;
            cmdTurnSpeed_dps = 0f;
            safeMove(0f);
            safeTurn(0f);
        }
    }

    public void moveContinuous(float speedMps) {
        synchronized (robotLock) {
            cmdMoveSpeed_mps = speedMps;
            safeMove(speedMps);
        }
    }

    public void turnContinuous(float dps) {
        synchronized (robotLock) {
            cmdTurnSpeed_dps = dps;
            safeTurn(dps);
        }
    }

    public void runMoveForSeconds(float speedMps, float seconds) {
        if (seconds <= 0f) return;
        moveContinuous(speedMps);
        uiHandler.postDelayed(this::stopMotion, (long) (seconds * 1000));
    }

    public void runTurnForSeconds(float dps, float seconds) {
        if (seconds <= 0f) return;
        turnContinuous(dps);
        uiHandler.postDelayed(this::stopMotion, (long) (seconds * 1000));
    }

    public void runMoveByDistance(float distanceMeters) {
        float speed = distanceMeters >= 0 ? DEFAULT_MOVE_SPEED : -DEFAULT_MOVE_SPEED;
        float time = Math.abs(distanceMeters / DEFAULT_MOVE_SPEED);
        runMoveForSeconds(speed, time);
    }

    public void runTurnByAngle(float degrees) {
        float speed = degrees >= 0 ? DEFAULT_TURN_SPEED : -DEFAULT_TURN_SPEED;
        float time = Math.abs(degrees / DEFAULT_TURN_SPEED);
        runTurnForSeconds(speed, time);
    }

    public void enterSleep() {
        callRobot("enterSleep", new Class[]{}, new Object[]{});
    }

    public void exitSleep() {
        callRobot("exitSleep", new Class[]{}, new Object[]{});
    }

    public void setAlwaysWakeup(boolean on) {
        callRobot("setAlwaysWakeup", new Class[]{boolean.class}, new Object[]{on});
    }

    public void setVoiceTrigger(boolean enable) {
        callRobot("setVoiceTrigger", new Class[]{boolean.class}, new Object[]{enable});
    }

    public void handleCoordinateCommand(String cmd) {
        if (cmd == null || cmd.isEmpty()) return;
        String lower = cmd.trim().toLowerCase(Locale.US);
        switch (lower) {
            case "forward":
                moveForward();
                break;
            case "backward":
                moveBackward();
                break;
            case "left":
                turnLeft();
                break;
            case "right":
                turnRight();
                break;
            case "stop":
                stopMotion();
                break;
            default:
                if (lower.startsWith("move:")) {
                    try {
                        float dist = Float.parseFloat(lower.substring(5));
                        runMoveByDistance(dist);
                    } catch (NumberFormatException ignore) {}
                } else if (lower.startsWith("turn:")) {
                    try {
                        float ang = Float.parseFloat(lower.substring(5));
                        runTurnByAngle(ang);
                    } catch (NumberFormatException ignore) {}
                }
                break;
        }
    }

    /* ------------ Nuwa Robot API ------------ */
    @Nullable
    private NuwaRobotAPI initNuwaApiCompat() {
        try {
            return new NuwaRobotAPI(appContext, null);
        } catch (Throwable t) {
            Log.e(TAG, "initNuwaApiCompat error", t);
            return null;
        }
    }

    private void registerRobotListenerSafe() {
        callRobot("registerRobotEventListener",
                new Class[]{RobotEventListener.class},
                new Object[]{robotListener});
    }

    private void unregisterRobotListenerSafe() {
        callRobot("unregisterRobotEventListener",
                new Class[]{RobotEventListener.class},
                new Object[]{robotListener});
        callRobot("removeRobotEventListener",
                new Class[]{RobotEventListener.class},
                new Object[]{robotListener});
    }

    private void registerVoiceListenerSafe() {
        callRobot("registerVoiceEventListener",
                new Class[]{VoiceEventListener.class},
                new Object[]{voiceListener});
    }

    private void unregisterVoiceListenerSafe() {
        callRobot("unregisterVoiceEventListener",
                new Class[]{VoiceEventListener.class},
                new Object[]{voiceListener});
        callRobot("removeVoiceEventListener",
                new Class[]{VoiceEventListener.class},
                new Object[]{voiceListener});
    }

    private void callRobot(String name, Class<?>[] sig, Object[] args) {
        if (robotApi == null) return;
        try {
            java.lang.reflect.Method m = NuwaRobotAPI.class.getMethod(name, sig);
            m.invoke(robotApi, args);
        } catch (NoSuchMethodException ns) {
            Log.w(TAG, "RobotAPI method missing: " + name);
        } catch (Throwable t) {
            Log.e(TAG, "RobotAPI call error: " + name, t);
        }
    }

    private final RobotEventListener robotListener = new RobotEventListener() {
        @Override
        public void onWikiServiceStart() {
            Log.d(TAG, "SDK Ready");
            setAlwaysWakeup(true);
        }

        @Override
        public void onMotorErrorEvent(int id, int code) {
            if (motionListener != null) {
                motionListener.onRobotEvent("ERR:MOTOR:" + id + ":" + code);
            }
        }

        @Override public void onWikiServiceStop() {}
        @Override public void onWikiServiceCrash() {}
        @Override public void onWikiServiceRecovery() {}
        @Override public void onStartOfMotionPlay(String s) {}
        @Override public void onPauseOfMotionPlay(String s) {}
        @Override public void onStopOfMotionPlay(String s) {}
        @Override public void onCompleteOfMotionPlay(String s) {}
        @Override public void onPlayBackOfMotionPlay(String s) {}
        @Override public void onErrorOfMotionPlay(int i) {}
        @Override public void onPrepareMotion(boolean ready, String motion, float progress) {}
        @Override public void onCameraOfMotionPlay(String motion) {}
        @Override public void onGetCameraPose(float f1, float f2, float f3,
                                              float f4, float f5, float f6, float f7,
                                              float f8, float f9, float f10,
                                              float f11, float f12) {}
        @Override public void onTouchEvent(int part, int type) {}
        @Override public void onPIREvent(int value) {}
        @Override public void onTap(int key) {}
        @Override public void onLongPress(int key) {}
        @Override public void onWindowSurfaceReady() {}
        @Override public void onWindowSurfaceDestroy() {}
        @Override public void onTouchEyes(int eye, int type) {}
        @Override public void onRawTouch(int x, int y, int type) {}
        @Override public void onFaceSpeaker(float angle) {}
        @Override public void onActionEvent(int action, int state) {
            Log.d(TAG, "Action event: action=" + action + ", state=" + state);
        }
        @Override public void onDropSensorEvent(int sensorId) {
            Log.d(TAG, "Drop sensor event: " + sensorId);
        }
    };

    private final VoiceEventListener voiceListener = new VoiceEventListener() {
        @Override public void onWakeup(boolean b, String s, float v) {}
        @Override public void onTTSComplete(boolean b) {}
        @Override public void onSpeechRecognizeComplete(boolean b, ResultType resultType, String s) {}
        @Override public void onSpeech2TextComplete(boolean b, String s) {}
        @Override public void onMixUnderstandComplete(boolean b, ResultType resultType, String s) {}
        @Override public void onSpeechState(ListenType listenType, SpeechState speechState) {}
        @Override public void onSpeakState(SpeakType speakType, SpeakState speakState) {}
        @Override public void onGrammarState(boolean b, String s) {}
        @Override public void onListenVolumeChanged(ListenType listenType, int i) {}
        @Override public void onHotwordChange(HotwordState hotwordState, HotwordType hotwordType, String s) {}
    };

    /* ------------ 里程計 ------------ */
    private void startPositionTimer() {
        if (posTimer != null) return;
        lastOdoNanos = System.nanoTime();
        posTimer = Executors.newSingleThreadScheduledExecutor();
        posTimer.scheduleAtFixedRate(() -> {
            try {
                long now = System.nanoTime();
                long last = lastOdoNanos;
                lastOdoNanos = now;
                float dt = (last == 0 ? 0f : (now - last) / 1_000_000_000f);
                if (dt <= 0f) return;

                int row, col;
                float x, z, hd;
                synchronized (robotLock) {
                    headingDeg += cmdTurnSpeed_dps * dt;
                    float rad = (float) Math.toRadians(headingDeg);
                    float v = cmdMoveSpeed_mps;
                    posZ_m += v * dt * (float) Math.cos(rad);
                    posX_m += v * dt * (float) Math.sin(rad);
                    x = posX_m;
                    z = posZ_m;
                    hd = headingDeg;
                    int[] rc = metersToCell(x, z);
                    row = rc[0];
                    col = rc[1];
                }
                if (motionListener != null) {
                    motionListener.onOdometryUpdate(row, col, x, z, hd);
                }
            } catch (Exception ignore) {}
        }, 0, 100, TimeUnit.MILLISECONDS);
    }

    private void stopPositionTimer() {
        if (posTimer != null) {
            posTimer.shutdownNow();
            posTimer = null;
        }
    }

    private static int[] metersToCell(float x, float z) {
        int col = (int) Math.floor((x - ROI_X_MIN) / CELL_M);
        int row = (int) Math.floor((ROI_Z_MAX - z) / CELL_M);
        col = clamp(col, 0, GRID_SIZE - 1);
        row = clamp(row, 0, GRID_SIZE - 1);
        return new int[]{row, col};
    }

    private static int clamp(int v, int lo, int hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private void safeMove(float speed) {
        try {
            if (robotApi != null) robotApi.move(speed);
        } catch (Throwable t) {
            Log.e(TAG, "move()", t);
        }
    }

    private void safeTurn(float dps) {
        try {
            if (robotApi != null) robotApi.turn(dps);
        } catch (Throwable t) {
            Log.e(TAG, "turn()", t);
        }
    }
}
