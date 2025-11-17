package com.kebbi.myapplication12;
import android.os.*;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.util.Log;
import androidx.annotation.Nullable;
import androidx.appcompat.app.AppCompatActivity;

import com.nuwarobotics.service.IClientId;
import com.nuwarobotics.service.agent.NuwaRobotAPI;
import com.nuwarobotics.service.agent.RobotEventListener;
import com.nuwarobotics.service.agent.VoiceEventListener;
import java.io.*;
import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.net.ServerSocket;
import java.net.Socket;
import android.os.Handler;
import android.os.Looper;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;
/**
 * 無 UI 版 MainActivity（*純時間控制版本*）
 * ---------------------------------------------------------
 * 1. 維護 (x,y,θ) 里程計（仍保留，可自行擴充）
 * 2. Always Wakeup 一直開著（除非你改）
 * 3. TCP 指令：sleep / wake / always:on|off
 * 4. 透過 TCP 指令，以「速度模式 + 秒數」控制機器人移動／旋轉
 *    - forward / backward / left / right    : 連續啟動速度
 *    - stop                                 : 完全停止
 *    - forwardTime:<t> / backTime:<t>       : 以固定線速度跑 *t* 秒再停
 *    - leftTime:<t> / rightTime:<t>         : 以固定角速度轉 *t* 秒再停
 *    - move:<d> / turn:<a>（距離 / 角度）   : 若仍需要 SDK 距離模式，可保留使用
 * ---------------------------------------------------------
 */
public class MainActivity extends AppCompatActivity {

    private static final String TAG = "Main";
    private static final int    TCP_PORT        = 8888;

    /** 你觀測到的「固定線速度」與「固定角速度」, 單位: m/s, deg/s */
    private static final float  RAW_MOVE_SPEED  = 0.1747f;   // ← 實測後改這裡
    private static final float  RAW_TURN_SPEED  =  30.053f;     // ← 實測後改這裡

    /* ====== ★ 和 Jetson 對齊的棋盤設定（4×4 m，320×320 格） ====== */
    private static final int   GRID_SIZE   = 320;
    private static final float ROI_X_MIN   = -2f;
    private static final float ROI_X_MAX   =  2f;
    private static final float ROI_Z_MIN   =  0f;
    private static final float ROI_Z_MAX   =  4f;
    private static final float CELL_M      = (ROI_X_MAX - ROI_X_MIN) / GRID_SIZE; // 4/320=0.0125
    private NuwaRobotAPI mRobot;

    /* 里程計（如有外部定位可覆蓋；此處做簡易積分示意） */
    private volatile float posX_m = 0f;        // x 右為正
    private volatile float posZ_m = 0f;        // z 前為正
    private volatile float headingDeg = 0f;    // -∞~+∞，不強制夾角
    // 目前命令速度（給定速積分）
    private volatile float cmdMoveSpeed_mps = 0f;
    private volatile float cmdTurnSpeed_dps = 0f;

    /* 里程計（如有外部定位可覆蓋；此處做簡易積分示意） */
    private volatile float posX_m = 0f;        // x 右為正
    private volatile float posZ_m = 0f;        // z 前為正
    private volatile float headingDeg = 0f;    // -∞~+∞，不強制夾角
    // 目前命令速度（給定速積分）
    private volatile float cmdMoveSpeed_mps = 0f;
    private volatile float cmdTurnSpeed_dps = 0f;

    /* 你原本保留的成員（可當 UI 或除錯用，不影響上報） */
    private float currentX = 0f;
    private float currentY = 0f;
    private float currentAngle = 0f;

    private final Object motionLock = new Object();
    private volatile boolean motionDone = false;

    /* ---- 新增：清醒/睡眠控制 ---- */
    private volatile boolean mySleep = false;              // 軟睡眠旗標
    private final Handler ui = new Handler(Looper.getMainLooper());
    /* ====== TCP Server ====== */
    private volatile PrintWriter writer;
    private final Object writerLock = new Object();
    private Thread tcpThread;
    /* ====== 定時上報 / 积分 ====== */
    private ScheduledExecutorService posTimer;
    private volatile long lastOdoNanos = 0L;

    /* ----------------------------------------------------- */
    /* Android Life‑cycle                                    */
    /* ----------------------------------------------------- */
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        // setContentView(R.layout.main); // 無 UI

        IClientId id = new IClientId("com.kebbi.myapplication12");
        mRobot = new NuwaRobotAPI(this, id);
        mRobot.registerRobotEventListener(robotListener);

        startTCPServer();
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        stopPositionTimer();
        stopTCPServer();
        if (mRobot != null) {
            try { mRobot.unregisterRobotEventListener(robotListener); } catch (Throwable ignore) {}
            try { mRobot.unregisterVoiceEventListener(voiceListener); } catch (Throwable ignore) {}
            try { mRobot.release(); } catch (Throwable ignore) {}
        }
    }

    /* ----------------------------------------------------- */
    /* Robot 事件監聽                                         */
    /* ----------------------------------------------------- */
    private final RobotEventListener robotListener = new RobotEventListener() {
        @Override public void onWikiServiceStart() {
            Log.d(TAG, "SDK Ready");
            ui.post(() -> mRobot.controlAlwaysWakeup(true));
            mRobot.registerVoiceEventListener(voiceListener);
        }
        @Override public void onStopOfMotionPlay(String s) {
            synchronized (motionLock) {
                motionDone = true;
                motionLock.notifyAll();
            }
        }
        @Override public void onMotorErrorEvent(int id, int code) {
            Log.e(TAG, "MotorError id=" + id + " code=" + code);
            sendLineSafe("ERR:MOTOR:" + id + ":" + code);
        }
        /* 其餘 callback 留空實作 */
         public void onWikiServiceStop(){} public void onWikiServiceCrash(){}
        public void onWikiServiceRecovery(){} public void onStartOfMotionPlay(String s){}
        public void onPauseOfMotionPlay(String s){} public void onCompleteOfMotionPlay(String s){}
        public void onPlayBackOfMotionPlay(String s){} public void onErrorOfMotionPlay(int i){}
        public void onPrepareMotion(boolean b,String s,float v){} public void onCameraOfMotionPlay(String s){}
        public void onGetCameraPose(float a,float b,float c,float d,float e,float f,float g,float h,float i,float j,float k,float l){}
        public void onTouchEvent(int a,int b){} public void onPIREvent(int i){}
        public void onTap(int i){} public void onLongPress(int i){} public void onWindowSurfaceReady(){}
        public void onWindowSurfaceDestroy(){} public void onTouchEyes(int a,int b){}
        public void onRawTouch(int a,int b,int c){} public void onFaceSpeaker(float v){}
        public void onActionEvent(int a,int b){} public void onDropSensorEvent(int i){}
    };

    /* 語音喚醒監聽（你原本那段 onWakeup） */
    /* 語音喚醒監聽（保持空實作或妳自行補） */
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

    /* ----------------------------------------------------- */
    /* TCP 伺服器                                             */
    /* ----------------------------------------------------- */
    private void startTCPServer() {
        if (tcpThread != null) return;
        tcpThread = new Thread(() -> {
            try (ServerSocket serverSocket = new ServerSocket(TCP_PORT)) {
                Log.d(TAG, "TCP Server Started on " + TCP_PORT);
                while (!Thread.currentThread().isInterrupted()) {
                    try (Socket client = serverSocket.accept()) {
                        Log.d(TAG, "TCP client connected: " + client.getInetAddress());
                        BufferedReader in = new BufferedReader(new InputStreamReader(client.getInputStream()));
                        synchronized (writerLock) {
                            writer = new PrintWriter(new BufferedWriter(new OutputStreamWriter(client.getOutputStream())), true);
                        }
                        startPositionTimerIfNeeded();
                        sendLineSafe("HELLO:RobotReady");

                        String line;
                        while ((line = in.readLine()) != null) {
                            handleSingleCommand(line.trim());
                        }
                    } catch (Exception e) {
                        Log.e(TAG, "TCP client error", e);
                    } finally {
                        synchronized (writerLock) { writer = null; }
                        stopPositionTimer();
                        Log.d(TAG, "TCP client disconnected");
                    }
                }
            } catch (Exception e) {
                Log.e(TAG, "TCP Server fatal", e);
            }
        }, "tcp-server");
        tcpThread.start();
    }

    private void stopTCPServer() {
        if (tcpThread != null) {
            tcpThread.interrupt();
            tcpThread = null;
        }
    }

    private void sendLineSafe(String s) {
        synchronized (writerLock) {
            if (writer != null) {
                writer.println(s);
                writer.flush();
            }
        }
    }
    /* ----------------------------------------------------- */
    /* 位置積分與定時回報（100ms）                            */
    /* ----------------------------------------------------- */
    private void startPositionTimerIfNeeded() {
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

                // 積分（簡化模型；若有 IMU/編碼器，換成實測）
                headingDeg += cmdTurnSpeed_dps * dt;
                float rad = (float) Math.toRadians(headingDeg);
                float v = cmdMoveSpeed_mps;
                posZ_m += v * dt * (float) Math.cos(rad); // 前 (+Z)
                posX_m += v * dt * (float) Math.sin(rad); // 右 (+X)

                int[] rc = metersToCell(posX_m, posZ_m);
                sendLineSafe("POS:" + rc[0] + "," + rc[1]);
                sendLineSafe(String.format(java.util.Locale.US, "POSM:%.3f,%.3f", posX_m, posZ_m));
            } catch (Exception ignore) {}
        }, 0, 100, TimeUnit.MILLISECONDS);
    }

    private void stopPositionTimer() {
        if (posTimer != null) {
            posTimer.shutdownNow();
            posTimer = null;
        }
    }

    private static int clamp(int v, int lo, int hi){ return Math.max(lo, Math.min(hi, v)); }
    private static int[] metersToCell(float x, float z) {
        int col = (int) Math.floor((x - ROI_X_MIN) / CELL_M);
        int row = (int) Math.floor((ROI_Z_MAX - z) / CELL_M); // z 越大 row 越小
        col = clamp(col, 0, GRID_SIZE - 1);
        row = clamp(row, 0, GRID_SIZE - 1);
        return new int[]{ row, col };
    }


    private static int clamp(int v, int lo, int hi){ return Math.max(lo, Math.min(hi, v)); }
    private static int[] metersToCell(float x, float z) {
        int col = (int) Math.floor((x - ROI_X_MIN) / CELL_M);
        int row = (int) Math.floor((ROI_Z_MAX - z) / CELL_M); // z 越大 row 越小
        col = clamp(col, 0, GRID_SIZE - 1);
        row = clamp(row, 0, GRID_SIZE - 1);
        return new int[]{ row, col };
    }
    /* ----------------------------------------------------- */
    /* 指令解析                                               */
    /* ----------------------------------------------------- */
    private void handleCommand(String cmd){
        // 先處理清醒/睡眠相關
        if ("sleep".equalsIgnoreCase(cmd)) {             // 進入軟睡眠
            enterMySleep();
            return;
        } else if ("wake".equalsIgnoreCase(cmd)) {       // 解除軟睡眠
            exitMySleep();
            return;
        } else if (cmd.toLowerCase().startsWith("always:")) { // always:on / always:off
            boolean on = cmd.toLowerCase().endsWith("on");
            ui.post(() -> mRobot.controlAlwaysWakeup(on));
            Log.d(TAG, "AlwaysWakeup -> " + on);
            return;
        }

        // 再執行你原本的 goto / 多指令分拆邏輯
        if(cmd.toLowerCase().startsWith("goto:")){
            handleCoordinateCommand(cmd.substring(5));
            return;
        }
        for(String c : cmd.split(";")){
            handleSingleCommand(c.trim().toLowerCase());
        }
    }

    /* ----------------------------------------------------- */
    /* 軟睡眠（不影響系統熱詞，純安靜化）                       */
    /* ----------------------------------------------------- */
    private void enterMySleep() {
        mySleep = true;
        try {
            ui.post(() -> {
                mRobot.stopListen();   // 停止語音流程
                mRobot.hideFace();     // 隱藏臉
                mRobot.lockWheel();    // 鎖輪避免誤動
            });
            Log.d(TAG, ">> enterMySleep()");
        } catch (Exception e) {
            Log.e(TAG, "enterMySleep err", e);
        }
    }

    private void exitMySleep() {
        mySleep = false;
        try {
            ui.post(() -> {
                mRobot.showFace();
                mRobot.unlockWheel();
                // 想醒來就主動開始聽可打開：
                // mRobot.startRecognize(false);
            });
            Log.d(TAG, ">> exitMySleep()");
        } catch (Exception e) {
            Log.e(TAG, "exitMySleep err", e);
        }
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        try {
            ui.post(() -> mRobot.controlAlwaysWakeup(true));
        } catch (Exception ignore) {}
        mRobot.release();
    }


    /**
     * 處理單一指令（小寫）
     */
    private void handleSingleCommand(String c) {
        try {
            c = c.trim().toLowerCase();

            // ---- 連續控制：速度模式（需 stop 才停）----
            switch (c) {
                case "forward":
                case "fwd":
                    Log.d(TAG, "➡ 收到 forward 指令");
                    cmdMoveSpeed_mps = +RAW_MOVE_SPEED;    // ★ 里程整合用
                    mRobot.move(+RAW_MOVE_SPEED);
                    sendLineSafe("ACK:forward");
                    return;
                case "backward":
                case "back":
                    Log.d(TAG, "⬅ 收到 backward 指令");
                    cmdMoveSpeed_mps = -RAW_MOVE_SPEED;
                    mRobot.move(-RAW_MOVE_SPEED);
                    sendLineSafe("ACK:backward");
                    return;
                case "left":
                    Log.d(TAG, "⟲ 收到 left 指令");
                    cmdTurnSpeed_dps = -RAW_TURN_SPEED;    // 左轉為負
                    mRobot.turn(-RAW_TURN_SPEED);
                    sendLineSafe("ACK:left");
                    return;
                case "right":
                    Log.d(TAG, "⟳ 收到 right 指令");
                    cmdTurnSpeed_dps = +RAW_TURN_SPEED;
                    mRobot.turn(+RAW_TURN_SPEED);
                    sendLineSafe("ACK:right");
                    return;
                case "stop":
                    Log.d(TAG, "🛑 收到 stop 指令，停止所有運動");
                    cmdMoveSpeed_mps = 0f;
                    cmdTurnSpeed_dps = 0f;
                    mRobot.move(0f);
                    mRobot.turn(0f);
                    // ⚠ 建議不要在 stop 時重設角度，否則會讓地圖定位跳掉
                    // currentAngle = 0f;  // ← 建議註解掉
                    sendLineSafe("ACK:stop");
                    return;
            }

            // ---- 秒數控制（保持，但也同步更新里程整合速度）----
            if (c.startsWith("forwardtime:")) {
                float t = Float.parseFloat(c.substring(12));
                Log.d(TAG, "➡ forwardTime 秒數: " + t);
                runMoveForSeconds(+RAW_MOVE_SPEED, t);
                return;
            } else if (c.startsWith("backtime:")) {
                float t = Float.parseFloat(c.substring(9));
                Log.d(TAG, "⬅ backTime 秒數: " + t);
                runMoveForSeconds(-RAW_MOVE_SPEED, t);
                return;
            } else if (c.startsWith("lefttime:")) {
                float t = Float.parseFloat(c.substring(9));
                Log.d(TAG, "⟲ leftTime 秒數: " + t);
                runTurnForSeconds(-RAW_TURN_SPEED, t);
                return;
            } else if (c.startsWith("righttime:")) {
                float t = Float.parseFloat(c.substring(10));
                Log.d(TAG, "⟳ rightTime 秒數: " + t);
                runTurnForSeconds(+RAW_TURN_SPEED, t);
                return;
            }

            // ---- 距離 / 角度控制：完成會自動停 ＋ ACK ----
            if (c.startsWith("move:")) {
                float d = Float.parseFloat(c.substring(5));
                Log.d(TAG, "🚶 move 距離: " + d + " 公尺");
                runMoveByDistance(d);
                return;
            } else if (c.startsWith("turn:")) {
                float a = Float.parseFloat(c.substring(5));
                Log.d(TAG, "🔁 turn 角度: " + a + " 度");
                runTurnByAngle(a);
                return;
            }

            Log.e(TAG, "❓ Unknown command: " + c);

        } catch (Exception e) {
            Log.e(TAG, "💥 handleSingleCommand 發生錯誤", e);
            sendLineSafe("ERR:" + e.getMessage());
        }
    }

    // 根據給定的速度與秒數移動（時間控制）
    private void handleSingleCommand(String c) {
        try {
            c = c.trim().toLowerCase();

            // ---- 連續控制：速度模式（需 stop 才停）----
            switch (c) {
                case "forward":
                case "fwd":
                    Log.d(TAG, "➡ 收到 forward 指令");
                    cmdMoveSpeed_mps = +RAW_MOVE_SPEED;    // ★ 里程整合用
                    mRobot.move(+RAW_MOVE_SPEED);
                    sendLineSafe("ACK:forward");
                    return;
                case "backward":
                case "back":
                    Log.d(TAG, "⬅ 收到 backward 指令");
                    cmdMoveSpeed_mps = -RAW_MOVE_SPEED;
                    mRobot.move(-RAW_MOVE_SPEED);
                    sendLineSafe("ACK:backward");
                    return;
                case "left":
                    Log.d(TAG, "⟲ 收到 left 指令");
                    cmdTurnSpeed_dps = -RAW_TURN_SPEED;    // 左轉為負
                    mRobot.turn(-RAW_TURN_SPEED);
                    sendLineSafe("ACK:left");
                    return;
                case "right":
                    Log.d(TAG, "⟳ 收到 right 指令");
                    cmdTurnSpeed_dps = +RAW_TURN_SPEED;
                    mRobot.turn(+RAW_TURN_SPEED);
                    sendLineSafe("ACK:right");
                    return;
                case "stop":
                    Log.d(TAG, "🛑 收到 stop 指令，停止所有運動");
                    cmdMoveSpeed_mps = 0f;
                    cmdTurnSpeed_dps = 0f;
                    mRobot.move(0f);
                    mRobot.turn(0f);
                    // ⚠ 建議不要在 stop 時重設角度，否則會讓地圖定位跳掉
                    // currentAngle = 0f;  // ← 建議註解掉
                    sendLineSafe("ACK:stop");
                    return;
            }

            // ---- 秒數控制（保持，但也同步更新里程整合速度）----
            if (c.startsWith("forwardtime:")) {
                float t = Float.parseFloat(c.substring(12));
                Log.d(TAG, "➡ forwardTime 秒數: " + t);
                runMoveForSeconds(+RAW_MOVE_SPEED, t);
                return;
            } else if (c.startsWith("backtime:")) {
                float t = Float.parseFloat(c.substring(9));
                Log.d(TAG, "⬅ backTime 秒數: " + t);
                runMoveForSeconds(-RAW_MOVE_SPEED, t);
                return;
            } else if (c.startsWith("lefttime:")) {
                float t = Float.parseFloat(c.substring(9));
                Log.d(TAG, "⟲ leftTime 秒數: " + t);
                runTurnForSeconds(-RAW_TURN_SPEED, t);
                return;
            } else if (c.startsWith("righttime:")) {
                float t = Float.parseFloat(c.substring(10));
                Log.d(TAG, "⟳ rightTime 秒數: " + t);
                runTurnForSeconds(+RAW_TURN_SPEED, t);
                return;
            }

            // ---- 距離 / 角度控制：完成會自動停 ＋ ACK ----
            if (c.startsWith("move:")) {
                float d = Float.parseFloat(c.substring(5));
                Log.d(TAG, "🚶 move 距離: " + d + " 公尺");
                runMoveByDistance(d);
                return;
            } else if (c.startsWith("turn:")) {
                float a = Float.parseFloat(c.substring(5));
                Log.d(TAG, "🔁 turn 角度: " + a + " 度");
                runTurnByAngle(a);
                return;
            }

            Log.e(TAG, "❓ Unknown command: " + c);

        } catch (Exception e) {
            Log.e(TAG, "💥 handleSingleCommand 發生錯誤", e);
            sendLineSafe("ERR:" + e.getMessage());
        }
    }

    /* ----------------------------------------------------- */
    /* 里程計輔助 & 座標導航（保留原邏輯，不含速度→時間換算） */
    /* ----------------------------------------------------- */


    public void handleCoordinateCommand(String input) {
        if (!input.contains(",")) {
            Log.e(TAG, "❌ 指令需含逗號: " + input);
            return;
        }
        try {
            String[] p = input.split(",");
            float tx = Float.parseFloat(p[0].trim());
            float ty = Float.parseFloat(p[1].trim());

            // ✅ 每次當前位置當原點，重設姿態！
            currentX = 0f;
            currentY = 0f;
            currentAngle = 0f;
            moveToCoordinate(tx, ty);
        } catch (Exception e) {
            Log.e(TAG, "⚠️ 座標格式錯誤: " + input, e);
        }
    }

    /** 讓機器人從目前位置移動到指定座標 */
    private void moveToCoordinate(float targetX, float targetY) {
        float dy = targetX - currentX;
        float dx = targetY - currentY;

        float targetAngle = (float) Math.toDegrees(Math.atan2(dy, dx));
        float turnAngle = wrapAngle(targetAngle - currentAngle);
        float distance = (float) Math.hypot(dx, dy);

        Log.d(TAG, "🧭 準備移動至目標座標: (" + targetX + ", " + targetY + ")");
        Log.d(TAG, "➡ 計算結果: 需轉向 " + turnAngle + "°，前進距離 " + distance + " m");

        new Thread(() -> {
            // 先旋轉
            runTurnByAngle(turnAngle);
            try {
                Thread.sleep((long) (Math.abs(turnAngle / RAW_TURN_SPEED) * 1000));
            } catch (InterruptedException ignored) {}

            // 再前進
            runMoveByDistance(distance);
            try {
                Thread.sleep((long) (Math.abs(distance / RAW_MOVE_SPEED) * 1000));
            } catch (InterruptedException ignored) {}

            // 更新位置
            currentX = targetX;
            currentY = targetY;
            currentAngle = targetAngle;
            Log.d(TAG, "📍 更新位置: (" + currentX + ", " + currentY + ") 朝向 " + currentAngle + "°");
        }).start();
    }

    /** 限制角度為 -180° ~ +180° */
    private float wrapAngle(float a) {
        if (a > 180f) return a - 360f;
        if (a < -180f) return a + 360f;
        return a;
    }
}