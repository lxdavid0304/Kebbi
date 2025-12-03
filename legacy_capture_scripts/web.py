
from flask import Flask, send_from_directory, send_file, request, jsonify
import os
import cv2

def create_app():
    app = Flask(__name__)
    
    app.display = None
    app.tcp_client = None  # 將由 main() 設定
    app.auto_drive_controls = {}
    
    @app.route('/')
    def index():
        static_dir = os.path.join(os.path.dirname(__file__), 'static')
        return send_file(os.path.join(static_dir, 'index.html'))

    @app.route('/static/<path:filename>')
    def static_files(filename):
        return send_from_directory('static', filename)
    
    @app.route('/current_view')
    def current_view():
        if app.display is not None:
            _, img_encoded = cv2.imencode('.jpg', app.display)
            return img_encoded.tobytes(), 200, {'Content-Type': 'image/jpg'}
        else:
            return "No image available", 404
    
    @app.route('/send_command', methods=['POST'])
    def send_command():
        """處理來自 Web UI 的控制命令"""
        try:
            data = request.get_json()
            command = data.get('command', '').lower()
            
            if not command:
                return jsonify({'success': False, 'message': '無效的命令'}), 400
            
            # 檢查 TCP 連線狀態
            if app.tcp_client is None:
                return jsonify({'success': False, 'message': 'TCP 未初始化'}), 503
            
            if not app.tcp_client.connected():
                return jsonify({'success': False, 'message': 'TCP 未連線，請確認機器人 IP 和 Port'}), 503
            
            # 根據命令發送對應的 TCP 指令
            command_map = {
                'w': ('move:0.5', '前進'),
                's': ('move:-0.5', '後退'),
                'a': ('turn:30', '左轉'),
                'd': ('turn:-30', '右轉'),
                'x': ('stop', '停止'),
            }
            
            if command not in command_map:
                return jsonify({'success': False, 'message': f'未知命令: {command}'}), 400

            tcp_cmd, desc = command_map[command]
            guard_checker = getattr(app, 'is_blind_guard_active', None)
            guard_blocked = callable(guard_checker) and guard_checker()
            if guard_blocked and tcp_cmd.lower() != 'stop':
                return jsonify({'success': False, 'message': '盲人距離過遠，暫停手動移動'}), 409
            app.tcp_client.send_line(tcp_cmd)
            print(f"[web] 發送命令: {desc} ({tcp_cmd})")
            
            return jsonify({'success': True, 'message': f'已發送: {desc}'})
            
        except Exception as exc:
            print(f"[web] 處理命令時發生錯誤: {exc}")
            return jsonify({'success': False, 'message': f'伺服器錯誤: {str(exc)}'}), 500

    @app.route('/auto_drive', methods=['GET', 'POST'])
    def auto_drive():
        controls = getattr(app, 'auto_drive_controls', {}) or {}
        getter = controls.get('get_status')
        setter = controls.get('set_enabled')

        try:
            if request.method == 'GET':
                if not getter:
                    return jsonify({'success': False, 'message': '自動導航狀態尚未就緒'}), 503
                return jsonify({'success': True, 'status': getter()})

            data = request.get_json() or {}
            if 'enabled' not in data:
                return jsonify({'success': False, 'message': '缺少 enabled 參數'}), 400
            if not setter:
                return jsonify({'success': False, 'message': '自動導航控制尚未就緒'}), 503

            enabled_value = data.get('enabled')
            if isinstance(enabled_value, bool):
                enabled = enabled_value
            elif isinstance(enabled_value, str):
                enabled = enabled_value.strip().lower() in ('1', 'true', 'yes', 'on')
            elif isinstance(enabled_value, (int, float)):
                enabled = bool(enabled_value)
            else:
                return jsonify({'success': False, 'message': 'enabled 參數格式錯誤'}), 400

            status = setter(enabled)
            message = '已啟用自動導航' if enabled else '已停用自動導航'
            print(f"[web] {message}")
            return jsonify({'success': True, 'message': message, 'status': status})
        except Exception as exc:
            print(f"[web] 處理自動導航控制錯誤: {exc}")
            return jsonify({'success': False, 'message': f'伺服器錯誤: {str(exc)}'}), 500
    
    return app
