package com.kebbi.myapplication12;

import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.util.Log;

import androidx.appcompat.app.AppCompatActivity;

import com.kebbi.myapplication12.net.RobotTcpServer;
import com.kebbi.myapplication12.robot.RobotMotionController;

import java.util.Locale;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public class MainActivity extends AppCompatActivity {

    private static final String TAG = "Main";
    private static final int TCP_PORT = 8888;

    private final Handler ui = new Handler(Looper.getMainLooper());
    private final ExecutorService tcpExecutor = Executors.newSingleThreadExecutor();
    private RobotMotionController motionController;
    private RobotTcpServer tcpServer;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        motionController = new RobotMotionController(
                this,
                ui,
                new RobotMotionController.MotionListener() {
                    @Override
                    public void onOdometryUpdate(int row, int col, float xMeters, float zMeters, float headingDeg) {
                        sendTcpLine(String.format(Locale.US, "POS:%d,%d", row, col));
                        sendTcpLine(String.format(Locale.US, "POSM:%.3f,%.3f,%.1f", xMeters, zMeters, headingDeg));
                    }

                    @Override
                    public void onRobotEvent(String message) {
                        sendTcpLine(message);
                    }
                }
        );
        motionController.start();

        tcpServer = new RobotTcpServer(
                TCP_PORT,
                ui,
                this::handleCommand,
                new RobotTcpServer.ConnectionListener() {
                    @Override
                    public void onClientConnected() {
                        sendTcpLine("HELLO:RobotReady");
                    }

                    @Override
                    public void onClientDisconnected() {
                        Log.d(TAG, "TCP client disconnected");
                    }
                }
        );
        tcpServer.start();
    }

    @Override
    protected void onDestroy() {
        if (tcpServer != null) {
            tcpServer.stop();
            tcpServer = null;
        }
        tcpExecutor.shutdownNow();
        if (motionController != null) {
            motionController.stop();
            motionController = null;
        }
        super.onDestroy();
    }

    private void handleCommand(String cmd) {
        if (cmd == null || cmd.isEmpty()) {
            return;
        }
        String lower = cmd.toLowerCase(Locale.US);
        if ("sleep".equals(lower)) {
            motionController.enterSleep();
            return;
        } else if ("wake".equals(lower)) {
            motionController.exitSleep();
            return;
        } else if (lower.startsWith("always:")) {
            final boolean on = lower.endsWith("on");
            motionController.setAlwaysWakeup(on);
            Log.d(TAG, "AlwaysWakeup -> " + on);
            sendTcpLine("ACK:always:" + (on ? "on" : "off"));
            return;
        }

        if (lower.startsWith("goto:")) {
            motionController.handleCoordinateCommand(cmd.substring(5));
            return;
        }
        for (String part : cmd.split(";")) {
            handleSingleCommand(part.trim().toLowerCase(Locale.US));
        }
    }

    private void handleSingleCommand(String c) {
        try {
            switch (c) {
                case "forward":
                case "fwd":
                    Log.d(TAG, "➡ forward");
                    motionController.moveForward();
                    sendTcpLine("ACK:forward");
                    return;
                case "backward":
                case "back":
                    Log.d(TAG, "⬅ backward");
                    motionController.moveBackward();
                    sendTcpLine("ACK:backward");
                    return;
                case "left":
                    Log.d(TAG, "⟲ left");
                    motionController.turnLeft();
                    sendTcpLine("ACK:left");
                    return;
                case "right":
                    Log.d(TAG, "⟳ right");
                    motionController.turnRight();
                    sendTcpLine("ACK:right");
                    return;
                case "stop":
                    Log.d(TAG, "🛑 stop");
                    motionController.stopMotion();
                    sendTcpLine("ACK:stop");
                    return;
            }

            if (c.startsWith("forwardtime:")) {
                float t = Float.parseFloat(c.substring(12));
                motionController.runMoveForSeconds(RobotMotionController.DEFAULT_MOVE_SPEED, t);
                return;
            } else if (c.startsWith("backtime:")) {
                float t = Float.parseFloat(c.substring(9));
                motionController.runMoveForSeconds(-RobotMotionController.DEFAULT_MOVE_SPEED, t);
                return;
            } else if (c.startsWith("lefttime:")) {
                float t = Float.parseFloat(c.substring(9));
                motionController.runTurnForSeconds(-RobotMotionController.DEFAULT_TURN_SPEED, t);
                return;
            } else if (c.startsWith("righttime:")) {
                float t = Float.parseFloat(c.substring(10));
                motionController.runTurnForSeconds(RobotMotionController.DEFAULT_TURN_SPEED, t);
                return;
            }

            if (c.startsWith("move:")) {
                float d = Float.parseFloat(c.substring(5));
                motionController.runMoveByDistance(d);
                return;
            } else if (c.startsWith("turn:")) {
                float a = Float.parseFloat(c.substring(5));
                motionController.runTurnByAngle(a);
                return;
            }

            Log.e(TAG, "❓ Unknown command: " + c);
            sendTcpLine("ERR:UNKNOWN:" + c);
        } catch (Exception e) {
            Log.e(TAG, "handleSingleCommand error", e);
            sendTcpLine("ERR:" + e.getMessage());
        }
    }

    private void sendTcpLine(String line) {
        if (tcpServer != null && line != null) {
            tcpExecutor.execute(() -> tcpServer.sendLine(line));
        }
    }
}
