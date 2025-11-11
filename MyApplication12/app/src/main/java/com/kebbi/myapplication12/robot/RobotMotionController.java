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
    private float headingDeg = 0f;
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

    public void runMoveForSeconds(final float speed, final float seconds) {
        new Thread(() -> {
            try {
                moveContinuous(speed);
                Thread.sleep((long) (Math.abs(seconds) * 1000));
            } catch (InterruptedException ignored) {
            } finally {
                stopMotion();
            }
        }, "move-seconds").start();
    }

    public void runTurnForSeconds(final float angularSpeed, final float seconds) {
        new Thread(() -> {
            try {
                turnContinuous(angularSpeed);
                Thread.sleep((long) (Math.abs(seconds) * 1000));
            } catch (InterruptedException ignored) {
            } finally {
                stopMotion();
            }
        }, "turn-seconds").start();
    }

    public void runMoveByDistance(final float distanceM) {
        float seconds = Math.abs(distanceM) / DEFAULT_MOVE_SPEED;
        float speed = (distanceM >= 0 ? +1f : -1f) * DEFAULT_MOVE_SPEED;
        runMoveForSeconds(speed, seconds);
    }

    public void runTurnByAngle(final float degrees) {
        float seconds = Math.abs(degrees) / DEFAULT_TURN_SPEED;
        float angular = (degrees >= 0 ? +1f : -1f) * DEFAULT_TURN_SPEED;
        runTurnForSeconds(angular, seconds);
    }

    public void handleCoordinateCommand(String input) {
        if (!input.contains(",")) {
            Log.e(TAG, "❌ 指令需含逗號: " + input);
            return;
        }
        try {
            String[] p = input.split(",");
            float tx = Float.parseFloat(p[0].trim());
            float tz = Float.parseFloat(p[1].trim());
            moveToCoordinate(0f, 0f, 0f, tx, tz);
        } catch (Exception e) {
            Log.e(TAG, "⚠ 座標格式錯誤: " + input, e);
        }
    }

    private void moveToCoordinate(float curX, float curZ, float curDeg,
                                  float targetX, float targetZ) {
        float dx = targetX - curX;
        float dz = targetZ - curZ;

        float targetAngle = (float) Math.toDegrees(Math.atan2(dx, dz));
        float turnAngle = wrapAngle(targetAngle - curDeg);
        float distance = (float) Math.hypot(dx, dz);

        Log.d(TAG, String.format(Locale.US,
                "🧭 goto (%.2f,%.2f) -> turn %.1f°, move %.2fm",
                targetX, targetZ, turnAngle, distance));

        new Thread(() -> {
            runTurnByAngle(turnAngle);
            try {
                Thread.sleep((long) (Math.abs(turnAngle / DEFAULT_TURN_SPEED) * 1000));
            } catch (InterruptedException ignored) {
            }
            runMoveByDistance(distance);
        }, "goto").start();
    }

    private float wrapAngle(float a) {
        while (a > 180f) a -= 360f;
        while (a < -180f) a += 360f;
        return a;
    }

    /* ------------ 睡眠/喚醒 ------------ */
    public void enterSleep() {
        uiHandler.post(() -> {
            try {
                synchronized (robotLock) {
                    if (robotApi != null) {
                        robotApi.stopListen();
                        robotApi.hideFace();
                        robotApi.lockWheel();
                    }
                }
            } catch (Throwable t) {
                Log.e(TAG, "enterSleep err", t);
            }
        });
    }

    public void exitSleep() {
        uiHandler.post(() -> {
            try {
                synchronized (robotLock) {
                    if (robotApi != null) {
                        robotApi.showFace();
                        robotApi.unlockWheel();
                    }
                }
            } catch (Throwable t) {
                Log.e(TAG, "exitSleep err", t);
            }
        });
    }

    public void setAlwaysWakeup(boolean on) {
        synchronized (robotLock) {
            try {
                if (robotApi != null) {
                    robotApi.controlAlwaysWakeup(on);
                }
            } catch (Throwable t) {
                Log.e(TAG, "AlwaysWakeup error", t);
            }
        }
    }

    /* ------------ Nuwa API ------------ */
    @Nullable
    private NuwaRobotAPI initNuwaApiCompat() {
        try {
            // 1) (Context, String)
            try {
                java.lang.reflect.Constructor<NuwaRobotAPI> c =
                        NuwaRobotAPI.class.getConstructor(android.content.Context.class, String.class);
                return c.newInstance(appContext, appContext.getPackageName());
            } catch (Throwable ignore) {}

            // 2) (Context)
            try {
                java.lang.reflect.Constructor<NuwaRobotAPI> c =
                        NuwaRobotAPI.class.getConstructor(android.content.Context.class);
                return c.newInstance(appContext);
            } catch (Throwable ignore) {}

            // 3) getInstance(Context)
            try {
                java.lang.reflect.Method m =
                        NuwaRobotAPI.class.getMethod("getInstance", android.content.Context.class);
                Object api = m.invoke(null, appContext);
                return (NuwaRobotAPI) api;
            } catch (Throwable ignore) {}

            // 4) createInstance(Context, String)
            try {
                java.lang.reflect.Method m =
                        NuwaRobotAPI.class.getMethod("createInstance",
                                android.content.Context.class, String.class);
                Object api = m.invoke(null, appContext, appContext.getPackageName());
                return (NuwaRobotAPI) api;
            } catch (Throwable ignore) {}

            Log.e(TAG, "無法建立 NuwaRobotAPI 實例");
            return null;
        } catch (Throwable t) {
            Log.e(TAG, "initNuwaApiCompat exception", t);
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
        callRobot("addVoiceEventListener",
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
