# 🔧 Kebbi機器人TCP控制問題診斷

## 📋 問題描述
- Android Studio端運行My Application正常
- 執行`client.py`下指令通過TCP傳送給機器人
- TCP連線成功，但機器人不會移動

## 🔍 已發現的問題

### 1. ❌ 測試腳本使用錯誤的指令名稱
**問題文件**: [`test_tcp_commands.py`](test_tcp_commands.py:45-47)

**錯誤指令**:
- `turn_left` ❌
- `turn_right` ❌

**正確指令** (已修正):
- `left` ✅
- `right` ✅

**Android端支援的指令**: 參考 [`MainActivity.java`](nuwa_android_agent/app/src/main/java/com/kebbi/myapplication12/MainActivity.java:112-168)
```java
case "forward":
case "backward": 
case "left":
case "right":
case "stop":
```

---

### 2. ⚠️ Android端NuwaRobotAPI可能未正確初始化

**檢查點**:
1. [`RobotMotionController.start()`](nuwa_android_agent/app/src/main/java/com/kebbi/myapplication12/robot/RobotMotionController.java:71-78) 是否成功執行
2. [`initNuwaApiCompat()`](nuwa_android_agent/app/src/main/java/com/kebbi/myapplication12/robot/RobotMotionController.java:208-214) 是否返回非null
3. [`onWikiServiceStart()`](nuwa_android_agent/app/src/main/java/com/kebbi/myapplication12/robot/RobotMotionController.java:261-264) 是否被觸發

**可能的問題**:
- NuwaRobotAPI初始化失敗（返回null）
- Robot Service未啟動
- 權限不足

---

### 3. 🚫 Android端缺少的功能

#### (A) 座標導航功能未完整實現
**問題**: [`RobotMotionController.handleCoordinateCommand()`](nuwa_android_agent/app/src/main/java/com/kebbi/myapplication12/robot/RobotMotionController.java:171-204) 只處理基本移動指令，不支援座標格式。

**Python端期望**: 
```python
# client.py 會將 "1.0,0.5" 轉換為 "goto:1.0,0.5"
if "," in line and not low.startswith("goto:"):
    line = "goto:" + line
```

**Android端現況**:
```java
// MainActivity.java 收到 goto: 後轉發
if (lower.startsWith("goto:")) {
    motionController.handleCoordinateCommand(cmd.substring(5));
    return;
}

// 但 RobotMotionController.handleCoordinateCommand() 無法處理 "x,y" 格式
// 只能處理 "move:0.5" 或 "turn:45" 這類單一參數指令
```

**建議**: 需要在Android端實現座標導航邏輯

---

#### (B) 心跳回應機制
Python端 [`RobotTCPClient`](realsense_planner/robot_tcp.py:134-149) 會發送PING，期待收到PONG回應。

**Android端需要添加**:
```java
// 在 MainActivity.handleCommand() 添加
if ("PING".equals(cmd)) {
    sendTcpLine("PONG");
    return;
}
```

---

## 🧪 診斷步驟

### 步驟 1: 執行基本測試
```bash
python3 test_simple_movement.py
```

**期望看到**:
```
✅ TCP連線成功！
📩 收到回應: HELLO:RobotReady
📤 已發送: forward
📩 收到回應: ACK:forward
📩 收到回應: POS:160,160
📩 收到回應: POSM:0.000,0.000,90.0
```

### 步驟 2: 檢查Android Logcat
在Android Studio開啟Logcat，篩選TAG:
- `RobotMotion`
- `RobotTcpServer`
- `Main`

**查看是否有**:
```
✅ SDK Ready
✅ ➡ forward
✅ TCP server started on port 8888
✅ Client connected: ...
```

**檢查是否有錯誤**:
```
❌ initNuwaApiCompat error
❌ move() error
❌ RobotAPI call error
```

### 步驟 3: 測試直接移動指令
使用[`client.py`](client.py)互動模式:
```bash
python3 client.py
>> forward
>> stop
>> backward
>> stop
```

---

## 🔑 關鍵檢查點

### Android端必須確認：

1. **✓ MainActivity已啟動且在前台**
   - TCP Server在`onCreate()`中啟動
   - 如果app在背景，可能被系統暫停

2. **✓ NuwaRobotAPI成功初始化**
   ```java
   // 在 RobotMotionController.java 添加日誌
   if (robotApi != null) {
       Log.d(TAG, "✅ NuwaRobotAPI initialized");
   } else {
       Log.e(TAG, "❌ NuwaRobotAPI initialization FAILED");
   }
   ```

3. **✓ Robot Service已啟動**
   - 查看`onWikiServiceStart()`是否被調用
   - 這是NuwaRobotAPI準備就緒的標誌

4. **✓ 移動指令確實被執行**
   ```java
   // 在每個移動函數添加日誌
   public void moveForward() {
       Log.d(TAG, "moveForward() called, speed=" + DEFAULT_MOVE_SPEED);
       moveContinuous(DEFAULT_MOVE_SPEED);
   }
   ```

5. **✓ robotApi.move() 和 robotApi.turn() 沒有拋出異常**
   - 檢查[`safeMove()`](nuwa_android_agent/app/src/main/java/com/kebbi/myapplication12/robot/RobotMotionController.java:372-378)和[`safeTurn()`](nuwa_android_agent/app/src/main/java/com/kebbi/myapplication12/robot/RobotMotionController.java:380-386)的catch區塊

---

## 💡 建議的修正

### 修正1: 在Android端添加PING/PONG處理
在[`MainActivity.handleCommand()`](nuwa_android_agent/app/src/main/java/com/kebbi/myapplication12/MainActivity.java:82-108)添加：

```java
private void handleCommand(String cmd) {
    if (cmd == null || cmd.isEmpty()) {
        return;
    }
    
    // 添加 PING/PONG 處理
    if ("PING".equals(cmd)) {
        sendTcpLine("PONG");
        return;
    }
    
    String lower = cmd.toLowerCase(Locale.US);
    // ... 其他處理
}
```

### 修正2: 添加詳細的錯誤日誌
在關鍵位置添加日誌：

```java
// RobotMotionController.java
private void safeMove(float speed) {
    try {
        if (robotApi != null) {
            Log.d(TAG, "🚗 Calling robotApi.move(" + speed + ")");
            robotApi.move(speed);
            Log.d(TAG, "✅ robotApi.move() executed successfully");
        } else {
            Log.e(TAG, "❌ robotApi is NULL, cannot move!");
        }
    } catch (Throwable t) {
        Log.e(TAG, "❌ move() exception", t);
    }
}
```

### 修正3: 確保app保持喚醒
在`MainActivity.onCreate()`添加：

```java
@Override
protected void onCreate(Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);
    
    // 保持螢幕開啟
    getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
    
    // ... 其他初始化
}
```

---

## 📱 Android端完整檢查清單

- [ ] App正在前台運行
- [ ] Logcat顯示 "TCP server started on port 8888"
- [ ] Logcat顯示 "Client connected"
- [ ] Logcat顯示 "SDK Ready"（NuwaRobotAPI已就緒）
- [ ] 收到指令時有 "➡ forward" 等日誌
- [ ] 沒有 "robotApi is NULL" 錯誤
- [ ] 沒有 "move() exception" 錯誤
- [ ] 機器人的輪子沒有被鎖定（非sleep模式）
- [ ] 機器人電池有電

---

## 🎯 下一步行動

1. **立即執行**: `python3 test_simple_movement.py`
2. **查看Android Logcat**: 找出實際錯誤訊息
3. **根據Logcat結果**: 確定是初始化問題還是執行問題
4. **如需要**: 在Android端添加更多日誌輸出
5. **回報**: 將Logcat的關鍵訊息提供給我，我可以進一步協助

---

## 📞 常見問題

**Q: TCP連線成功但沒有任何回應？**
A: 可能Android app沒在前台或TCP Server沒啟動，重啟app試試。

**Q: 收到ACK但機器人不動？**
A: 檢查NuwaRobotAPI是否初始化成功（查看Logcat的"SDK Ready"）。

**Q: client.py能控制但Python的RobotTCPClient不行？**
A: 可能是PING/PONG機制問題，需要在Android端添加PONG回應。

**Q: 機器人移動一下就停？**
A: 可能是指令執行時間太短，嘗試用`forwardTime:3.0`這類計時指令。