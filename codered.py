###!/usr/bin/env python3
"""Raspberry Pi 5 ESP32 Swarm Monitor with MAX7219 LED Matrix
Receives data from ESP32 swarm, displays on web interface, controls LEDs, logs data,
and shows photocell trace on 8x8 LED matrix"""

import socket
import json
import threading
import time
from datetime import datetime
from flask import Flask, render_template, jsonify, send_from_directory
from collections import deque, defaultdict
import os
import lgpio
import spidev

# ===== Configuration =====
UDP_PORT = 5005
WEB_PORT = 8080
LOG_DIR = "swarm_logs"

# GPIO Pin Configuration (BCM numbering)
LED_ESP1 = 18      # GPIO18 - LED for ESP 1 (master indicator)
LED_ESP2 = 23      # GPIO23 - LED for ESP 2 (master indicator)
LED_ESP3 = 24      # GPIO24 - LED for ESP 3 (master indicator)
LED_YELLOW = 25    # GPIO25 - Yellow LED (button press indicator)
BUTTON_PIN = 21    # GPIO21 - Reset button

# GPIO chip handle
gpio_chip = None

# SPI for MAX7219 LED Matrix
spi = None

# MAX7219 Register addresses
MAX7219_REG_NOOP = 0x00
MAX7219_REG_DIGIT0 = 0x01
MAX7219_REG_DIGIT1 = 0x02
MAX7219_REG_DIGIT2 = 0x03
MAX7219_REG_DIGIT3 = 0x04
MAX7219_REG_DIGIT4 = 0x05
MAX7219_REG_DIGIT5 = 0x06
MAX7219_REG_DIGIT6 = 0x07
MAX7219_REG_DIGIT7 = 0x08
MAX7219_REG_DECODEMODE = 0x09
MAX7219_REG_INTENSITY = 0x0A
MAX7219_REG_SCANLIMIT = 0x0B
MAX7219_REG_SHUTDOWN = 0x0C
MAX7219_REG_DISPLAYTEST = 0x0F

# Data structures
data_lock = threading.Lock()
photocell_history = deque(maxlen=300)  # ~30 seconds at 10Hz
master_durations = defaultdict(float)
current_master = None
master_start_time = None
esp_to_led = {}  # Will map IP addresses to LED pins
known_esps = []  # Track discovered ESPs in order

# LED Matrix data - store last 8 columns (each ~4 seconds)
matrix_data = deque(maxlen=8)  # Each entry is average light value over ~4 seconds
matrix_accumulator = []
matrix_last_update = 0
MATRIX_UPDATE_INTERVAL = 4.0  # Update matrix every 4 seconds

# Current session data
session_start_time = None
current_log_data = []

# Console output control
last_console_update = 0
CONSOLE_UPDATE_INTERVAL = 2.0  # Print status every 2 seconds

# Button handling
button_thread_running = True
last_button_state = 1

# Flask app
app = Flask(__name__)

# ===== MAX7219 LED Matrix Functions =====
def setup_spi():
    """Initialize SPI for MAX7219"""
    global spi
    spi = spidev.SpiDev()
    spi.open(0, 0)  # SPI bus 0, device 0 (CE0)
    spi.max_speed_hz = 1000000
    spi.mode = 0
    print("SPI initialized for MAX7219")

def max7219_write(register, data):
    """Write to MAX7219 register"""
    spi.xfer2([register, data])

def max7219_init():
    """Initialize MAX7219 LED matrix"""
    max7219_write(MAX7219_REG_SHUTDOWN, 0x00)      # Shutdown mode
    max7219_write(MAX7219_REG_DISPLAYTEST, 0x00)   # Normal operation
    max7219_write(MAX7219_REG_DECODEMODE, 0x00)    # No decode for all digits
    max7219_write(MAX7219_REG_SCANLIMIT, 0x07)     # Scan all 8 digits
    max7219_write(MAX7219_REG_INTENSITY, 0x08)     # Medium brightness (0-15)
    max7219_write(MAX7219_REG_SHUTDOWN, 0x01)      # Normal operation
    
    # Clear display
    for i in range(8):
        max7219_write(MAX7219_REG_DIGIT0 + i, 0x00)
    
    print("MAX7219 LED matrix initialized")

def update_led_matrix():
    """Update LED matrix with photocell trace data"""
    # Create 8x8 display buffer
    display = [0] * 8  # 8 columns
    
    # Fill columns with stored data (right to left, newest on right)
    for col_idx, avg_value in enumerate(matrix_data):
        # Map average value (0-4095) to height (0-8)
        height = int((avg_value / 4095.0) * 8)
        height = max(0, min(8, height))  # Clamp to 0-8
        
        # Create column pattern (bottom to top)
        col_pattern = 0
        for row in range(height):
            col_pattern |= (1 << row)
        
        display[col_idx] = col_pattern
    
    # Send to MAX7219 (columns are digits 0-7)
    for col in range(8):
        if col < len(display):
            max7219_write(MAX7219_REG_DIGIT0 + col, display[col])
        else:
            max7219_write(MAX7219_REG_DIGIT0 + col, 0x00)

def clear_led_matrix():
    """Clear the LED matrix display"""
    for i in range(8):
        max7219_write(MAX7219_REG_DIGIT0 + i, 0x00)
    print("LED matrix cleared")

# ===== GPIO Setup for Raspberry Pi 5 =====
def setup_gpio():
    """Initialize GPIO pins using lgpio for Raspberry Pi 5"""
    global gpio_chip
    
    # Open GPIO chip (gpiochip4 for Raspberry Pi 5)
    gpio_chip = lgpio.gpiochip_open(4)
    
    # Setup LED outputs
    lgpio.gpio_claim_output(gpio_chip, LED_ESP1)
    lgpio.gpio_claim_output(gpio_chip, LED_ESP2)
    lgpio.gpio_claim_output(gpio_chip, LED_ESP3)
    lgpio.gpio_claim_output(gpio_chip, LED_YELLOW)
    
    # Turn off all LEDs
    lgpio.gpio_write(gpio_chip, LED_ESP1, 0)
    lgpio.gpio_write(gpio_chip, LED_ESP2, 0)
    lgpio.gpio_write(gpio_chip, LED_ESP3, 0)
    lgpio.gpio_write(gpio_chip, LED_YELLOW, 0)
    
    # Setup button with pull-up
    lgpio.gpio_claim_input(gpio_chip, BUTTON_PIN, lgpio.SET_PULL_UP)
    
    print("GPIO initialized successfully")

def cleanup_gpio():
    """Clean up GPIO resources"""
    global gpio_chip, spi
    if gpio_chip is not None:
        lgpio.gpiochip_close(gpio_chip)
        print("GPIO cleaned up")
    if spi is not None:
        spi.close()
        print("SPI closed")

# ===== LED Control =====
def update_master_leds(master_ip):
    """Update LEDs to show which ESP is master"""
    lgpio.gpio_write(gpio_chip, LED_ESP1, 0)
    lgpio.gpio_write(gpio_chip, LED_ESP2, 0)
    lgpio.gpio_write(gpio_chip, LED_ESP3, 0)
    
    if master_ip and master_ip in esp_to_led:
        led_pin = esp_to_led[master_ip]
        lgpio.gpio_write(gpio_chip, led_pin, 1)

def blink_yellow_led():
    """Blink yellow LED for 3 seconds"""
    lgpio.gpio_write(gpio_chip, LED_YELLOW, 1)
    time.sleep(3)
    lgpio.gpio_write(gpio_chip, LED_YELLOW, 0)

# ===== Button Handler (Polling) =====
def button_monitor():
    """Monitor button state in separate thread"""
    global last_button_state, button_thread_running
    
    while button_thread_running:
        try:
            # Read button state (LOW when pressed due to pull-up)
            button_state = lgpio.gpio_read(gpio_chip, BUTTON_PIN)
            
            # Detect falling edge (button press)
            if last_button_state == 1 and button_state == 0:
                handle_button_press()
                time.sleep(0.3)  # Debounce
            
            last_button_state = button_state
            time.sleep(0.01)  # Poll every 10ms
            
        except Exception as e:
            print(f"Button monitor error: {e}")
            time.sleep(0.1)

def handle_button_press():
    """Handle button press event"""
    print("\n" + "="*60)
    print("BUTTON PRESSED - RESET SEQUENCE INITIATED")
    print("="*60)
    
    # Save current log file
    print("Saving current log file...")
    saved_file = save_log_file()
    if saved_file:
        print(f"Log saved: {saved_file}")
    else:
        print("No data to save")
    
    # Blink yellow LED
    print("Blinking yellow LED for 3 seconds...")
    blink_thread = threading.Thread(target=blink_yellow_led)
    blink_thread.start()
    
    # Send reset to ESPs
    print("Broadcasting RESET command to all ESPs...")
    send_reset_command()
    print("RESET command sent")
    
    # Clear LED matrix
    clear_led_matrix()
    
    # Start new session
    print("Starting new logging session...")
    start_new_session()
    print("New session started")
    
    print("="*60)
    print("RESET SEQUENCE COMPLETE")
    print("="*60 + "\n")

# ===== Logging =====
def create_log_directory():
    """Create log directory if it doesn't exist"""
    if not os.path.exists(LOG_DIR):
        os.makedirs(LOG_DIR)
        print(f"Created log directory: {LOG_DIR}")
    else:
        print(f"Log directory exists: {LOG_DIR}")

def save_log_file():
    """Save current session data to a log file"""
    global current_log_data, session_start_time
    
    if not current_log_data:
        return None
    
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"swarm_log_{timestamp}.json"
    filepath = os.path.join(LOG_DIR, filename)
    
    # Calculate final master durations
    final_durations = dict(master_durations)
    
    log_entry = {
        "session_start": session_start_time,
        "session_end": datetime.now().isoformat(),
        "master_durations": final_durations,
        "data_points": current_log_data,
        "total_data_points": len(current_log_data)
    }
    
    try:
        with open(filepath, 'w') as f:
            json.dump(log_entry, f, indent=2)
        return filename
    except Exception as e:
        print(f"Error saving log: {e}")
        return None

def start_new_session():
    """Start a new logging session"""
    global session_start_time, current_log_data, master_durations
    global photocell_history, current_master, master_start_time, last_console_update
    global matrix_data, matrix_accumulator, matrix_last_update
    
    with data_lock:
        session_start_time = datetime.now().isoformat()
        current_log_data = []
        master_durations.clear()
        photocell_history.clear()
        current_master = None
        master_start_time = None
        last_console_update = 0
        matrix_data.clear()
        matrix_accumulator = []
        matrix_last_update = time.time()
    
    print(f"New session started at {session_start_time}")

# ===== UDP Data Reception =====
def udp_listener():
    """Listen for UDP packets from ESP32 devices"""
    global current_master, master_start_time, known_esps, esp_to_led, last_console_update
    global matrix_accumulator, matrix_last_update
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('', UDP_PORT))
    
    print(f"UDP listener started on port {UDP_PORT}")
    
    available_leds = [LED_ESP1, LED_ESP2, LED_ESP3]
    
    while True:
        try:
            data, addr = sock.recvfrom(1024)
            message = data.decode('utf-8').strip()
            
            # Parse JSON data from master ESP
            if message.startswith('{'):
                try:
                    packet = json.loads(message)
                    master_ip = packet.get('master')
                    value = packet.get('value')
                    
                    if master_ip and value is not None:
                        timestamp = datetime.now().isoformat()
                        
                        # Assign LED to new ESP if not already assigned
                        if master_ip not in esp_to_led and available_leds:
                            led_pin = available_leds.pop(0)
                            esp_to_led[master_ip] = led_pin
                            known_esps.append(master_ip)
                        
                        with data_lock:
                            # Update photocell history
                            photocell_history.append({
                                'timestamp': timestamp,
                                'value': value,
                                'master': master_ip
                            })
                            
                            # Log data point
                            current_log_data.append({
                                'timestamp': timestamp,
                                'master': master_ip,
                                'value': value
                            })
                            
                            # Accumulate data for LED matrix
                            matrix_accumulator.append(value)
                            
                            # Update LED matrix every 4 seconds
                            current_time = time.time()
                            if current_time - matrix_last_update >= MATRIX_UPDATE_INTERVAL:
                                if matrix_accumulator:
                                    avg_value = sum(matrix_accumulator) / len(matrix_accumulator)
                                    matrix_data.append(avg_value)
                                    matrix_accumulator = []
                                    matrix_last_update = current_time
                                    update_led_matrix()
                            
                            # Track master changes
                            if current_master != master_ip:
                                old_master = current_master
                                if current_master and master_start_time:
                                    duration = time.time() - master_start_time
                                    master_durations[current_master] += duration
                                
                                current_master = master_ip
                                master_start_time = time.time()
                                update_master_leds(master_ip)
                            
                            # Print periodic status update (throttled)
                            current_time = time.time()
                            if current_time - last_console_update >= CONSOLE_UPDATE_INTERVAL:
                                print(f"{master_ip} | Light: {value}\n", end='\r')
                                last_console_update = current_time
                                
                except json.JSONDecodeError:
                    pass
                
        except Exception as e:
            print(f"UDP listener error: {e}")

def send_reset_command():
    """Broadcast RESET command to all ESPs"""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    
    try:
        message = b"RESET"
        sock.sendto(message, ('<broadcast>', UDP_PORT))
    except Exception as e:
        print(f"Error sending reset: {e}")
    finally:
        sock.close()

# ===== Web Server Routes =====
@app.route('/')
def index():
    """Main dashboard page"""
    return render_template('dashboard.html')

@app.route('/api/current_data')
def get_current_data():
    """Get current photocell data and master info"""
    with data_lock:
        # Get recent data points (last 30 seconds)
        recent_data = list(photocell_history)
        
        # Calculate current master durations
        durations = dict(master_durations)
        if current_master and master_start_time:
            durations[current_master] = master_durations[current_master] + \
                                       (time.time() - master_start_time)
        
        return jsonify({
            'photocell_data': recent_data,
            'master_durations': durations,
            'current_master': current_master,
            'known_esps': known_esps,
            'session_start': session_start_time
        })

@app.route('/api/logs')
def get_logs():
    """Get list of available log files"""
    try:
        files = [f for f in os.listdir(LOG_DIR) if f.endswith('.json')]
        files.sort(reverse=True)
        return jsonify({'logs': files})
    except Exception as e:
        return jsonify({'logs': [], 'error': str(e)})

@app.route('/api/log/<filename>')
def get_log(filename):
    """Get specific log file data"""
    try:
        filepath = os.path.join(LOG_DIR, filename)
        with open(filepath, 'r') as f:
            data = json.load(f)
        return jsonify(data)
    except Exception as e:
        return jsonify({'error': str(e)}), 404

@app.route('/logs/<path:filename>')
def download_log(filename):
    """Download log file"""
    return send_from_directory(LOG_DIR, filename, as_attachment=True)

# ===== HTML Template =====
def create_html_template():
    """Create the dashboard HTML template"""
    template_dir = 'templates'
    if not os.path.exists(template_dir):
        os.makedirs(template_dir)
    
    html_content = '''<!DOCTYPE html>
<html>
<head>
    <title>ESP32 Swarm Monitor</title>
    <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
    <style>
        body {
            font-family: Arial, sans-serif;
            margin: 20px;
            background-color: #f5f5f5;
        }
        .container {
            max-width: 1400px;
            margin: 0 auto;
        }
        h1 {
            color: #333;
            text-align: center;
        }
        .mode-selector {
            background: white;
            padding: 15px;
            border-radius: 8px;
            margin-bottom: 20px;
            box-shadow: 0 2px 4px rgba(0,0,0,0.1);
            text-align: center;
        }
        .mode-btn {
            padding: 10px 30px;
            margin: 0 10px;
            border-radius: 4px;
            border: 2px solid #4CAF50;
            background-color: white;
            color: #4CAF50;
            cursor: pointer;
            font-size: 16px;
            font-weight: bold;
        }
        .mode-btn.active {
            background-color: #4CAF50;
            color: white;
        }
        .mode-btn:hover {
            background-color: #45a049;
            color: white;
        }
        .status {
            background: white;
            padding: 15px;
            border-radius: 8px;
            margin-bottom: 20px;
            box-shadow: 0 2px 4px rgba(0,0,0,0.1);
        }
        .charts {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 20px;
            margin-bottom: 20px;
        }
        .chart-container {
            background: white;
            padding: 20px;
            border-radius: 8px;
            box-shadow: 0 2px 4px rgba(0,0,0,0.1);
        }
        .log-section {
            background: white;
            padding: 20px;
            border-radius: 8px;
            box-shadow: 0 2px 4px rgba(0,0,0,0.1);
        }
        .file-upload-section {
            background: white;
            padding: 20px;
            border-radius: 8px;
            box-shadow: 0 2px 4px rgba(0,0,0,0.1);
            margin-bottom: 20px;
        }
        select, button, input[type="file"] {
            padding: 8px 15px;
            margin: 5px;
            border-radius: 4px;
            border: 1px solid #ddd;
        }
        button {
            background-color: #4CAF50;
            color: white;
            cursor: pointer;
        }
        button:hover {
            background-color: #45a049;
        }
        .master-indicator {
            display: inline-block;
            padding: 5px 10px;
            background-color: #4CAF50;
            color: white;
            border-radius: 4px;
            font-weight: bold;
        }
        .info-box {
            background: #e3f2fd;
            padding: 10px;
            border-radius: 4px;
            margin-top: 10px;
            font-size: 14px;
        }
        .offline-indicator {
            background: #ff9800;
            color: white;
            padding: 10px;
            border-radius: 4px;
            text-align: center;
            font-weight: bold;
            margin-bottom: 15px;
        }
        .online-indicator {
            background: #4CAF50;
            color: white;
            padding: 10px;
            border-radius: 4px;
            text-align: center;
            font-weight: bold;
            margin-bottom: 15px;
        }
        #offlineMode, #liveMode {
            display: none;
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>ESP32 Swarm Monitor Dashboard</h1>
        
        <div class="mode-selector">
            <button class="mode-btn active" id="liveModeBtn" onclick="switchMode('live')">Live Mode</button>
            <button class="mode-btn" id="offlineModeBtn" onclick="switchMode('offline')">Offline Mode</button>
        </div>

        <!-- LIVE MODE -->
        <div id="liveMode">
            <div id="connectionStatus" class="online-indicator">
                Connected - Receiving Live Data
            </div>
            
            <div class="status">
                <h3>Current Status</h3>
                <p><strong>Session Started:</strong> <span id="sessionStart">-</span></p>
                <p><strong>Current Master:</strong> <span id="currentMaster" class="master-indicator">None</span></p>
                <p><strong>Known ESPs:</strong> <span id="knownEsps">-</span></p>
                <div class="info-box">
                    <strong>LED Matrix:</strong> Displays photocell trace over last ~30 seconds (8 columns × 4 seconds each)
                </div>
            </div>
            
            <div class="log-section">
                <h3>Download Current Session Logs</h3>
                <select id="logSelect">
                    <option value="">Select a log file...</option>
                </select>
                <button onclick="downloadLog()">Download Selected Log</button>
            </div>
        </div>

        <!-- OFFLINE MODE -->
        <div id="offlineMode">
            <div class="offline-indicator">
                Offline Mode - Viewing Saved Log Data
            </div>
            <div class="file-upload-section">
                <h3>Load Log File</h3>
                <p><strong>Option 1:</strong> Upload a log file from your computer</p>
                <input type="file" id="fileInput" accept=".json" onchange="loadLocalFile()">
                
                <p style="margin-top: 20px;"><strong>Option 2:</strong> Load from Raspberry Pi (if available)</p>
                <select id="offlineLogSelect">
                    <option value="">Select a log file from Pi...</option>
                </select>
                <button onclick="loadPiLog()">Load from Pi</button>
            </div>
            <div class="status" id="offlineStatus" style="display: none;">
                <h3>Log File Information</h3>
                <p><strong>Session Start:</strong> <span id="offlineSessionStart">-</span></p>
                <p><strong>Session End:</strong> <span id="offlineSessionEnd">-</span></p>
                <p><strong>Total Data Points:</strong> <span id="offlineTotalPoints">-</span></p>
                <p><strong>Master ESPs:</strong> <span id="offlineMasters">-</span></p>
            </div>
        </div>

        <!-- CHARTS (Shared between modes) -->
        <div class="charts">
            <div class="chart-container">
                <h3>Photocell Data Over Time</h3>
                <canvas id="photocellChart"></canvas>
            </div>
            
            <div class="chart-container">
                <h3>Master Duration (seconds)</h3>
                <canvas id="masterChart"></canvas>
            </div>
        </div>
    </div>

    <script>
        let photocellChart, masterChart;
        let updateInterval;
        let currentMode = 'live';
        let isServerAvailable = true;
        let smoothedValues = []; // For smoothing light sensor readings

        // Simple moving average function for smoothing
        function movingAverage(values, windowSize = 3) {
            if (values.length < windowSize) return values;
            
            let smoothed = [];
            for (let i = 0; i < values.length; i++) {
                let start = Math.max(0, i - Math.floor(windowSize / 2));
                let end = Math.min(values.length, i + Math.ceil(windowSize / 2));
                let sum = 0;
                let count = 0;
                
                for (let j = start; j < end; j++) {
                    sum += values[j];
                    count++;
                }
                smoothed.push(sum / count);
            }
            return smoothed;
        }

        // Initialize charts
        function initCharts() {
            const photocellCtx = document.getElementById('photocellChart').getContext('2d');
            photocellChart = new Chart(photocellCtx, {
                type: 'line',
                data: {
                    labels: [],
                    datasets: [{
                        label: 'Light Value',
                        data: [],
                        borderColor: 'rgb(75, 192, 192)',
                        backgroundColor: 'rgba(75, 192, 192, 0.2)',
                        tension: 0.4,  // Increased from 0.1 for smoother curves
                        borderWidth: 2,
                        pointRadius: 1,  // Smaller points for cleaner look
                        pointHoverRadius: 4
                    }]
                },
                options: {
                    responsive: true,
                    scales: {
                        y: {
                            beginAtZero: true,
                            max: 4095,
                            title: { display: true, text: 'Photocell Value' }
                        },
                        x: {
                            title: { display: true, text: 'Time' }
                        }
                    },
                    animation: {
                        duration: 750,  // Smooth animation instead of instant
                        easing: 'easeInOutQuart'
                    },
                    transitions: {
                        active: {
                            animation: {
                                duration: 400
                            }
                        }
                    }
                }
            });
            
            const masterCtx = document.getElementById('masterChart').getContext('2d');
            masterChart = new Chart(masterCtx, {
                type: 'bar',
                data: {
                    labels: [],
                    datasets: [{
                        label: 'Duration (seconds)',
                        data: [],
                        backgroundColor: [
                            'rgba(255, 99, 132, 0.6)',
                            'rgba(54, 162, 235, 0.6)',
                            'rgba(255, 206, 86, 0.6)'
                        ]
                    }]
                },
                options: {
                    responsive: true,
                    scales: {
                        y: {
                            beginAtZero: true,
                            title: { display: true, text: 'Time as Master (s)' }
                        }
                    },
                    animation: {
                        duration: 500,
                        easing: 'easeInOutQuart'
                    }
                }
            });
        }

        // Switch between live and offline mode
        function switchMode(mode) {
            currentMode = mode;
            
            if (mode === 'live') {
                document.getElementById('liveMode').style.display = 'block';
                document.getElementById('offlineMode').style.display = 'none';
                document.getElementById('liveModeBtn').classList.add('active');
                document.getElementById('offlineModeBtn').classList.remove('active');
                
                // Clear charts and restart live updates
                clearCharts();
                updateCurrentData();  // Immediate update
                if (updateInterval) clearInterval(updateInterval);
                updateInterval = setInterval(updateCurrentData, 1000);  // Changed from 500ms to 1000ms for smoother updates
            } else {
                document.getElementById('liveMode').style.display = 'none';
                document.getElementById('offlineMode').style.display = 'block';
                document.getElementById('liveModeBtn').classList.remove('active');
                document.getElementById('offlineModeBtn').classList.add('active');
                
                // Stop live updates
                if (updateInterval) {
                    clearInterval(updateInterval);
                    updateInterval = null;
                }
                
                // Clear charts
                clearCharts();
                
                // Load available logs for offline mode
                loadOfflineLogList();
            }
        }

        // Clear all charts
        function clearCharts() {
            photocellChart.data.labels = [];
            photocellChart.data.datasets[0].data = [];
            photocellChart.update('none');  // Update without animation
            
            masterChart.data.labels = [];
            masterChart.data.datasets[0].data = [];
            masterChart.update('none');
            
            smoothedValues = [];
        }

        // Update charts with current live data
        function updateCurrentData() {
            fetch('/api/current_data')
                .then(response => {
                    if (!response.ok) throw new Error('Server not available');
                    return response.json();
                })
                .then(data => {
                    isServerAvailable = true;
                    document.getElementById('connectionStatus').className = 'online-indicator';
                    document.getElementById('connectionStatus').innerHTML = 'Connected - Receiving Live Data';
                    
                    // Update status
                    document.getElementById('sessionStart').textContent = 
                        data.session_start || 'No session';
                    document.getElementById('currentMaster').textContent = 
                        data.current_master || 'None';
                    document.getElementById('knownEsps').textContent = 
                        data.known_esps.join(', ') || 'None';
                    
                    // Update photocell chart with smoothing
                    const times = data.photocell_data.map(d => 
                        new Date(d.timestamp).toLocaleTimeString());
                    const rawValues = data.photocell_data.map(d => d.value);
                    
                    // Apply moving average smoothing
                    const smoothedValues = movingAverage(rawValues, 5);  // 5-point moving average
                    
                    photocellChart.data.labels = times;
                    photocellChart.data.datasets[0].data = smoothedValues;
                    photocellChart.update('active');  // Use active animation mode
                    
                    // Update master duration chart
                    const masters = Object.keys(data.master_durations);
                    const durations = Object.values(data.master_durations);
                    
                    masterChart.data.labels = masters;
                    masterChart.data.datasets[0].data = durations;
                    masterChart.update('active');
                })
                .catch(error => {
                    isServerAvailable = false;
                    document.getElementById('connectionStatus').className = 'offline-indicator';
                    document.getElementById('connectionStatus').innerHTML = 'Disconnected - No Live Data Available';
                });
        }

        // Load available log files for live mode
        function loadLogList() {
            fetch('/api/logs')
                .then(response => response.json())
                .then(data => {
                    const select = document.getElementById('logSelect');
                    select.innerHTML = '<option value="">Select a log file...</option>';
                    data.logs.forEach(log => {
                        const option = document.createElement('option');
                        option.value = log;
                        option.textContent = log;
                        select.appendChild(option);
                    });
                })
                .catch(error => {
                    console.log('Could not load logs from server');
                });
        }

        // Load available log files for offline mode
        function loadOfflineLogList() {
            fetch('/api/logs')
                .then(response => response.json())
                .then(data => {
                    const select = document.getElementById('offlineLogSelect');
                    select.innerHTML = '<option value="">Select a log file from Pi...</option>';
                    data.logs.forEach(log => {
                        const option = document.createElement('option');
                        option.value = log;
                        option.textContent = log;
                        select.appendChild(option);
                    });
                })
                .catch(error => {
                    console.log('Raspberry Pi not available - use file upload instead');
                    const select = document.getElementById('offlineLogSelect');
                    select.innerHTML = '<option value="">Raspberry Pi not connected</option>';
                });
        }

        // Load local file uploaded by user
        function loadLocalFile() {
            const fileInput = document.getElementById('fileInput');
            const file = fileInput.files[0];
            
            if (!file) return;
            
            const reader = new FileReader();
            reader.onload = function(e) {
                try {
                    const data = JSON.parse(e.target.result);
                    displayLogData(data);
                } catch (error) {
                    alert('Error reading file: ' + error.message);
                }
            };
            reader.readAsText(file);
        }

        // Load log from Raspberry Pi
        function loadPiLog() {
            const filename = document.getElementById('offlineLogSelect').value;
            if (!filename) {
                alert('Please select a log file');
                return;
            }
            
            fetch(`/api/log/${filename}`)
                .then(response => response.json())
                .then(data => {
                    displayLogData(data);
                })
                .catch(error => {
                    alert('Error loading log from Pi: ' + error.message);
                });
        }

        // Display log data in charts
        function displayLogData(data) {
            // Show status section
            document.getElementById('offlineStatus').style.display = 'block';
            
            // Update status info
            document.getElementById('offlineSessionStart').textContent = data.session_start || '-';
            document.getElementById('offlineSessionEnd').textContent = data.session_end || '-';
            document.getElementById('offlineTotalPoints').textContent = data.total_data_points || 0;
            
            const masters = Object.keys(data.master_durations || {});
            document.getElementById('offlineMasters').textContent = masters.join(', ') || 'None';
            
            // Update photocell chart with smoothing
            const times = data.data_points.map(d => 
                new Date(d.timestamp).toLocaleTimeString());
            const rawValues = data.data_points.map(d => d.value);
            
            // Apply moving average smoothing
            const smoothedValues = movingAverage(rawValues, 5);
            
            photocellChart.data.labels = times;
            photocellChart.data.datasets[0].data = smoothedValues;
            photocellChart.update();
            
            // Update master chart
            const masterLabels = Object.keys(data.master_durations || {});
            const durations = Object.values(data.master_durations || {});
            
            masterChart.data.labels = masterLabels;
            masterChart.data.datasets[0].data = durations;
            masterChart.update();
        }
        
        // Download log file
        function downloadLog() {
            const filename = document.getElementById('logSelect').value;
            if (!filename) {
                alert('Please select a log file to download');
                return;
            }
            window.location.href = `/logs/${filename}`;
        }
        
        // Initialize on page load
        window.onload = function() {
            initCharts();
            loadLogList();
            
            // Start in live mode and begin updates immediately
            switchMode('live');
        };
    </script>
</body>
</html>'''
    
    with open(os.path.join(template_dir, 'dashboard.html'), 'w') as f:
        f.write(html_content)
    
    print("HTML template created")

# ===== Main =====
def main():
    """Main application entry point"""
    global button_thread_running
    
    print("\n" + "="*60)
    print("ESP32 Swarm Monitor - Raspberry Pi 5")
    print("="*60)
    
    # Initialize hardware
    print("Initializing hardware...")
    setup_gpio()
    setup_spi()
    max7219_init()
    
    # Create log directory
    create_log_directory()
    
    # Create HTML template
    create_html_template()
    
    # Start new session
    start_new_session()
    
    # Start UDP listener thread
    print("Starting UDP listener...")
    udp_thread = threading.Thread(target=udp_listener, daemon=True)
    udp_thread.start()
    
    # Start button monitor thread
    print("Starting button monitor...")
    button_thread = threading.Thread(target=button_monitor, daemon=True)
    button_thread.start()
    
    # Start web server
    print(f"Starting web server on port {WEB_PORT}...")
    print(f"Dashboard: http://localhost:{WEB_PORT}")
    print("="*60 + "\n")
    
    try:
        app.run(host='0.0.0.0', port=WEB_PORT, debug=False, threaded=True)
    except KeyboardInterrupt:
        print("\n\nShutting down...")
        button_thread_running = False
        time.sleep(0.1)
        cleanup_gpio()
        print("Goodbye!\n")
        import sys
        sys.exit(0)

if __name__ == '__main__':
    main()
