package com.kebbi.myapplication12.net;

import android.os.Handler;
import android.util.Log;

import androidx.annotation.Nullable;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.io.PrintWriter;
import java.net.ServerSocket;
import java.net.Socket;

/**
 * 簡單的 TCP 伺服器：單一 client 連線，接收指令並可回傳字串。
 */
public class RobotTcpServer {

    private static final String TAG = "RobotTcpServer";

    public interface CommandHandler {
        void onCommand(String command);
    }

    public interface ConnectionListener {
        void onClientConnected();

        void onClientDisconnected();
    }

    private final int port;
    private final Handler callbackHandler;
    private final CommandHandler commandHandler;
    private final ConnectionListener connectionListener;

    private volatile boolean running = false;
    private Thread serverThread;
    private ServerSocket serverSocket;

    private final Object writerLock = new Object();
    @Nullable
    private PrintWriter writer;

    public RobotTcpServer(int port,
                          Handler callbackHandler,
                          CommandHandler handler,
                          ConnectionListener connectionListener) {
        this.port = port;
        this.callbackHandler = callbackHandler;
        this.commandHandler = handler;
        this.connectionListener = connectionListener;
    }

    public void start() {
        if (running) return;
        running = true;
        serverThread = new Thread(this::runServer, "robot-tcp-server");
        serverThread.start();
    }

    public void stop() {
        running = false;
        try {
            if (serverSocket != null) {
                serverSocket.close();
            }
        } catch (Exception ignore) {}
        synchronized (writerLock) {
            if (writer != null) {
                writer.close();
                writer = null;
            }
        }
        if (serverThread != null) {
            serverThread.interrupt();
            serverThread = null;
        }
    }

    public void sendLine(String line) {
        synchronized (writerLock) {
            if (writer != null) {
                writer.println(line);
                writer.flush();
            }
        }
    }

    private void runServer() {
        try (ServerSocket server = new ServerSocket(port)) {
            serverSocket = server;
            Log.d(TAG, "TCP server started on port " + port);
            while (running && !Thread.currentThread().isInterrupted()) {
                try {
                    Socket client = server.accept();
                    handleClient(client);
                } catch (Exception e) {
                    if (running) {
                        Log.e(TAG, "Client accept error", e);
                    }
                }
            }
        } catch (Exception e) {
            Log.e(TAG, "TCP server fatal", e);
        } finally {
            serverSocket = null;
        }
    }

    private void handleClient(Socket client) {
        Log.d(TAG, "Client connected: " + client.getInetAddress());
        try (Socket autoClose = client;
             BufferedReader reader = new BufferedReader(new InputStreamReader(autoClose.getInputStream()));
             PrintWriter out = new PrintWriter(
                     new BufferedWriter(new OutputStreamWriter(autoClose.getOutputStream())), true)) {

            synchronized (writerLock) {
                writer = out;
            }
            if (connectionListener != null) {
                callbackHandler.post(connectionListener::onClientConnected);
            }

            String line;
            while (running && (line = reader.readLine()) != null) {
                final String cmd = line.trim();
                if (!cmd.isEmpty()) {
                    callbackHandler.post(() -> commandHandler.onCommand(cmd));
                }
            }
        } catch (Exception e) {
            if (running) {
                Log.e(TAG, "Client error", e);
            }
        } finally {
            synchronized (writerLock) {
                writer = null;
            }
            if (connectionListener != null) {
                callbackHandler.post(connectionListener::onClientDisconnected);
            }
            Log.d(TAG, "Client disconnected");
        }
    }
}

