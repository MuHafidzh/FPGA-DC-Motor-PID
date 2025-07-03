import tkinter as tk
from tkinter import ttk, scrolledtext
import serial
import threading
import time
import queue
from datetime import datetime

class MotorControlGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Motor Control System - FPGA")
        self.root.geometry("900x700")
        
        # Serial connection
        self.ser = None
        self.connected = False
        
        # Data queues for thread communication
        self.encoder_queue = queue.Queue()
        self.log_queue = queue.Queue()
        
        # Encoder data storage
        self.encoder_data = {
            'motor0': {'position': 0, 'rpm': 0},
            'motor1': {'position': 0, 'rpm': 0}
        }
        
        # Control variables
        self.current_psc = 99
        self.current_ccr = 999
        self.current_ppr = 1100
        
        # Create GUI
        self.create_widgets()
        
        # Start background thread for serial communication
        self.serial_thread = None
        self.running = False
        
    def create_widgets(self):
        # Main container
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Connection frame
        conn_frame = ttk.LabelFrame(main_frame, text="Connection", padding="5")
        conn_frame.grid(row=0, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=5)
        
        ttk.Label(conn_frame, text="COM Port:").grid(row=0, column=0, padx=5)
        self.port_var = tk.StringVar(value="COM12")
        ttk.Entry(conn_frame, textvariable=self.port_var, width=10).grid(row=0, column=1, padx=5)
        
        self.connect_btn = ttk.Button(conn_frame, text="Connect", command=self.toggle_connection)
        self.connect_btn.grid(row=0, column=2, padx=5)
        
        self.status_label = ttk.Label(conn_frame, text="Disconnected", foreground="red")
        self.status_label.grid(row=0, column=3, padx=10)
        
        # Motor control frame
        motor_frame = ttk.LabelFrame(main_frame, text="Motor Control", padding="5")
        motor_frame.grid(row=1, column=0, sticky=(tk.W, tk.E, tk.N), padx=5, pady=5)
        
        # Motor 0
        ttk.Label(motor_frame, text="Motor 0 PWM:").grid(row=0, column=0, padx=5, pady=2)
        self.motor0_var = tk.StringVar(value="0")
        ttk.Entry(motor_frame, textvariable=self.motor0_var, width=10).grid(row=0, column=1, padx=5)
        
        # Motor 1
        ttk.Label(motor_frame, text="Motor 1 PWM:").grid(row=1, column=0, padx=5, pady=2)
        self.motor1_var = tk.StringVar(value="0")
        ttk.Entry(motor_frame, textvariable=self.motor1_var, width=10).grid(row=1, column=1, padx=5)
        
        ttk.Button(motor_frame, text="Send PWM", command=self.send_pwm).grid(row=2, column=0, columnspan=2, pady=5)
        ttk.Button(motor_frame, text="Stop Motors", command=self.stop_motors).grid(row=3, column=0, columnspan=2, pady=2)
        
        # Configuration frame
        config_frame = ttk.LabelFrame(main_frame, text="Configuration", padding="5")
        config_frame.grid(row=1, column=1, sticky=(tk.W, tk.E, tk.N), padx=5, pady=5)
        
        # PSC
        ttk.Label(config_frame, text="PSC:").grid(row=0, column=0, padx=5, pady=2)
        self.psc_var = tk.StringVar(value="99")
        ttk.Entry(config_frame, textvariable=self.psc_var, width=10).grid(row=0, column=1, padx=5)
        ttk.Button(config_frame, text="Set", command=lambda: self.send_config("psc")).grid(row=0, column=2, padx=2)
        
        # CCR
        ttk.Label(config_frame, text="CCR:").grid(row=1, column=0, padx=5, pady=2)
        self.ccr_var = tk.StringVar(value="999")
        ttk.Entry(config_frame, textvariable=self.ccr_var, width=10).grid(row=1, column=1, padx=5)
        ttk.Button(config_frame, text="Set", command=lambda: self.send_config("ccr")).grid(row=1, column=2, padx=2)
        
        # PPR
        ttk.Label(config_frame, text="PPR:").grid(row=2, column=0, padx=5, pady=2)
        self.ppr_var = tk.StringVar(value="1100")
        ttk.Entry(config_frame, textvariable=self.ppr_var, width=10).grid(row=2, column=1, padx=5)
        ttk.Button(config_frame, text="Set", command=lambda: self.send_config("ppr")).grid(row=2, column=2, padx=2)
        
        ttk.Button(config_frame, text="Query Settings", command=self.query_settings).grid(row=3, column=0, columnspan=3, pady=5)
        
        # PID Control frame
        pid_frame = ttk.LabelFrame(main_frame, text="PID Control", padding="5")
        pid_frame.grid(row=2, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=5)
        
        # Motor 0 PID
        ttk.Label(pid_frame, text="Motor 0:", font=("Arial", 10, "bold")).grid(row=0, column=0, columnspan=2, sticky=tk.W, pady=2)
        
        # Mode selection Motor 0
        ttk.Label(pid_frame, text="Mode:").grid(row=1, column=0, padx=5, sticky=tk.W)
        self.motor0_mode = tk.StringVar(value="PWM")
        ttk.Combobox(pid_frame, textvariable=self.motor0_mode, values=["PWM", "PID"], 
                    width=8, state="readonly").grid(row=1, column=1, padx=5)
        ttk.Button(pid_frame, text="Set Mode", 
                  command=lambda: self.send_pid_mode(0)).grid(row=1, column=2, padx=5)
        
        # PID Parameters Motor 0
        ttk.Label(pid_frame, text="Setpoint:").grid(row=2, column=0, padx=5, sticky=tk.W)
        self.motor0_setpoint = tk.StringVar(value="0")
        ttk.Entry(pid_frame, textvariable=self.motor0_setpoint, width=10).grid(row=2, column=1, padx=5)
        ttk.Button(pid_frame, text="Set", 
                  command=lambda: self.send_pid_param(0, "setpoint")).grid(row=2, column=2, padx=5)
        
        ttk.Label(pid_frame, text="Kp:").grid(row=3, column=0, padx=5, sticky=tk.W)
        self.motor0_kp = tk.StringVar(value="256")
        ttk.Entry(pid_frame, textvariable=self.motor0_kp, width=10).grid(row=3, column=1, padx=5)
        ttk.Button(pid_frame, text="Set", 
                  command=lambda: self.send_pid_param(0, "kp")).grid(row=3, column=2, padx=5)
        
        ttk.Label(pid_frame, text="Ki:").grid(row=4, column=0, padx=5, sticky=tk.W)
        self.motor0_ki = tk.StringVar(value="64")
        ttk.Entry(pid_frame, textvariable=self.motor0_ki, width=10).grid(row=4, column=1, padx=5)
        ttk.Button(pid_frame, text="Set", 
                  command=lambda: self.send_pid_param(0, "ki")).grid(row=4, column=2, padx=5)
        
        ttk.Label(pid_frame, text="Kd:").grid(row=5, column=0, padx=5, sticky=tk.W)
        self.motor0_kd = tk.StringVar(value="16")
        ttk.Entry(pid_frame, textvariable=self.motor0_kd, width=10).grid(row=5, column=1, padx=5)
        ttk.Button(pid_frame, text="Set", 
                  command=lambda: self.send_pid_param(0, "kd")).grid(row=5, column=2, padx=5)
        
        # Motor 1 PID (Column 3-5)
        ttk.Label(pid_frame, text="Motor 1:", font=("Arial", 10, "bold")).grid(row=0, column=3, columnspan=2, sticky=tk.W, pady=2, padx=20)
        
        # Mode selection Motor 1
        ttk.Label(pid_frame, text="Mode:").grid(row=1, column=3, padx=25, sticky=tk.W)
        self.motor1_mode = tk.StringVar(value="PWM")
        ttk.Combobox(pid_frame, textvariable=self.motor1_mode, values=["PWM", "PID"], 
                    width=8, state="readonly").grid(row=1, column=4, padx=5)
        ttk.Button(pid_frame, text="Set Mode", 
                  command=lambda: self.send_pid_mode(1)).grid(row=1, column=5, padx=5)
        
        # PID Parameters Motor 1
        ttk.Label(pid_frame, text="Setpoint:").grid(row=2, column=3, padx=25, sticky=tk.W)
        self.motor1_setpoint = tk.StringVar(value="0")
        ttk.Entry(pid_frame, textvariable=self.motor1_setpoint, width=10).grid(row=2, column=4, padx=5)
        ttk.Button(pid_frame, text="Set", 
                  command=lambda: self.send_pid_param(1, "setpoint")).grid(row=2, column=5, padx=5)
        
        ttk.Label(pid_frame, text="Kp:").grid(row=3, column=3, padx=25, sticky=tk.W)
        self.motor1_kp = tk.StringVar(value="256")
        ttk.Entry(pid_frame, textvariable=self.motor1_kp, width=10).grid(row=3, column=4, padx=5)
        ttk.Button(pid_frame, text="Set", 
                  command=lambda: self.send_pid_param(1, "kp")).grid(row=3, column=5, padx=5)
        
        ttk.Label(pid_frame, text="Ki:").grid(row=4, column=3, padx=25, sticky=tk.W)
        self.motor1_ki = tk.StringVar(value="64")
        ttk.Entry(pid_frame, textvariable=self.motor1_ki, width=10).grid(row=4, column=4, padx=5)
        ttk.Button(pid_frame, text="Set", 
                  command=lambda: self.send_pid_param(1, "ki")).grid(row=4, column=5, padx=5)
        
        ttk.Label(pid_frame, text="Kd:").grid(row=5, column=3, padx=25, sticky=tk.W)
        self.motor1_kd = tk.StringVar(value="16")
        ttk.Entry(pid_frame, textvariable=self.motor1_kd, width=10).grid(row=5, column=4, padx=5)
        ttk.Button(pid_frame, text="Set", 
                  command=lambda: self.send_pid_param(1, "kd")).grid(row=5, column=5, padx=5)
        
        # Encoder data frame with PID monitoring
        encoder_frame = ttk.LabelFrame(main_frame, text="Motor Status & RPM Monitoring (Real-time)", padding="5")
        encoder_frame.grid(row=3, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=5)
        
        # Motor 0 status and data
        ttk.Label(encoder_frame, text="Motor 0:", font=("Arial", 10, "bold")).grid(row=0, column=0, padx=5, sticky=tk.W)
        self.motor0_mode_label = ttk.Label(encoder_frame, text="Mode: PWM", foreground="blue")
        self.motor0_mode_label.grid(row=0, column=1, padx=10)
        self.enc0_pos_label = ttk.Label(encoder_frame, text="Position: 0")
        self.enc0_pos_label.grid(row=0, column=2, padx=10)
        self.enc0_rpm_label = ttk.Label(encoder_frame, text="RPM: 0", foreground="green")
        self.enc0_rpm_label.grid(row=0, column=3, padx=10)
        self.motor0_setpoint_label = ttk.Label(encoder_frame, text="Setpoint: 0")
        self.motor0_setpoint_label.grid(row=0, column=4, padx=10)
        self.motor0_error_label = ttk.Label(encoder_frame, text="Error: 0")
        self.motor0_error_label.grid(row=0, column=5, padx=10)
        
        # Motor 1 status and data
        ttk.Label(encoder_frame, text="Motor 1:", font=("Arial", 10, "bold")).grid(row=1, column=0, padx=5, sticky=tk.W)
        self.motor1_mode_label = ttk.Label(encoder_frame, text="Mode: PWM", foreground="blue")
        self.motor1_mode_label.grid(row=1, column=1, padx=10)
        self.enc1_pos_label = ttk.Label(encoder_frame, text="Position: 0")
        self.enc1_pos_label.grid(row=1, column=2, padx=10)
        self.enc1_rpm_label = ttk.Label(encoder_frame, text="RPM: 0", foreground="green")
        self.enc1_rpm_label.grid(row=1, column=3, padx=10)
        self.motor1_setpoint_label = ttk.Label(encoder_frame, text="Setpoint: 0")
        self.motor1_setpoint_label.grid(row=1, column=4, padx=10)
        self.motor1_error_label = ttk.Label(encoder_frame, text="Error: 0")
        self.motor1_error_label.grid(row=1, column=5, padx=10)
        
        # Log frame
        log_frame = ttk.LabelFrame(main_frame, text="Communication Log", padding="5")
        log_frame.grid(row=4, column=0, columnspan=2, sticky=(tk.W, tk.E, tk.N, tk.S), pady=5)
        
        self.log_text = scrolledtext.ScrolledText(log_frame, height=12, width=80)
        self.log_text.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        ttk.Button(log_frame, text="Clear Log", command=self.clear_log).grid(row=1, column=0, pady=5)
        
        # Configure grid weights
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(0, weight=1)
        main_frame.columnconfigure(1, weight=1)
        main_frame.rowconfigure(4, weight=1)  # Log frame is now at row 4
        log_frame.columnconfigure(0, weight=1)
        log_frame.rowconfigure(0, weight=1)
        
        # Initialize GUI state
        self.update_mode_display()
        
        # Start GUI update loop
        self.update_gui()
        
    def log_message(self, message, type="info"):
        timestamp = datetime.now().strftime("%H:%M:%S")
        color_map = {"info": "black", "success": "green", "error": "red", "warning": "orange"}
        color = color_map.get(type, "black")
        
        self.log_queue.put((f"[{timestamp}] {message}", color))
        
    def update_gui(self):
        # Update log
        try:
            while True:
                message, color = self.log_queue.get_nowait()
                self.log_text.insert(tk.END, message + "\n")
                # Scroll to bottom
                self.log_text.see(tk.END)
        except queue.Empty:
            pass
            
        # Update encoder data
        try:
            while True:
                enc_data = self.encoder_queue.get_nowait()
                self.encoder_data = enc_data
                
                # Update labels
                self.enc0_pos_label.config(text=f"Position: {enc_data['motor0']['position']:6d}")
                self.enc1_pos_label.config(text=f"Position: {enc_data['motor1']['position']:6d}")
                
                # Update RPM with color coding based on mode
                rpm0 = enc_data['motor0']['rpm']
                rpm1 = enc_data['motor1']['rpm']
                
                # Motor 0 RPM display
                if self.motor0_mode.get() == "PID":
                    try:
                        setpoint0 = int(self.motor0_setpoint.get()) if self.motor0_setpoint.get().lstrip('-').isdigit() else 0
                    except:
                        setpoint0 = 0
                    error0 = rpm0 - setpoint0  # Signed error
                    abs_error0 = abs(error0)
                    
                    if abs_error0 <= 5:  # Within ±5 RPM tolerance
                        rpm_color0 = "green"
                        error_color0 = "green"
                    elif abs_error0 <= 15:  # Within ±15 RPM tolerance
                        rpm_color0 = "orange"
                        error_color0 = "orange"
                    else:
                        rpm_color0 = "red"
                        error_color0 = "red"
                        
                    self.enc0_rpm_label.config(text=f"RPM: {rpm0:4d}", foreground=rpm_color0)
                    self.motor0_error_label.config(text=f"Error: {error0:+4d}", foreground=error_color0)
                else:
                    self.enc0_rpm_label.config(text=f"RPM: {rpm0:4d}", foreground="green")
                    self.motor0_error_label.config(text="Error: -")
                
                # Motor 1 RPM display
                if self.motor1_mode.get() == "PID":
                    try:
                        setpoint1 = int(self.motor1_setpoint.get()) if self.motor1_setpoint.get().lstrip('-').isdigit() else 0
                    except:
                        setpoint1 = 0
                    error1 = rpm1 - setpoint1  # Signed error
                    abs_error1 = abs(error1)
                    
                    if abs_error1 <= 5:  # Within ±5 RPM tolerance
                        rpm_color1 = "green"
                        error_color1 = "green"
                    elif abs_error1 <= 15:  # Within ±15 RPM tolerance
                        rpm_color1 = "orange"
                        error_color1 = "orange"
                    else:
                        rpm_color1 = "red"
                        error_color1 = "red"
                        
                    self.enc1_rpm_label.config(text=f"RPM: {rpm1:4d}", foreground=rpm_color1)
                    self.motor1_error_label.config(text=f"Error: {error1:+4d}", foreground=error_color1)
                else:
                    self.enc1_rpm_label.config(text=f"RPM: {rpm1:4d}", foreground="green")
                    self.motor1_error_label.config(text="Error: -")
                
                # Update mode display and setpoint visibility
                self.update_mode_display()
                
        except queue.Empty:
            pass
            
        # Schedule next update
        self.root.after(50, self.update_gui)  # Update every 50ms
        
    def toggle_connection(self):
        if not self.connected:
            try:
                self.ser = serial.Serial(self.port_var.get(), 115200, timeout=1.0)
                self.connected = True
                self.status_label.config(text="Connected", foreground="green")
                self.connect_btn.config(text="Disconnect")
                self.log_message(f"Connected to {self.port_var.get()}", "success")
                
                # Start serial thread
                self.running = True
                self.serial_thread = threading.Thread(target=self.serial_worker, daemon=True)
                self.serial_thread.start()
                
            except Exception as e:
                self.log_message(f"Connection failed: {e}", "error")
        else:
            self.disconnect()
            
    def disconnect(self):
        self.running = False
        if self.ser:
            self.ser.close()
        self.connected = False
        self.status_label.config(text="Disconnected", foreground="red")
        self.connect_btn.config(text="Connect")
        self.log_message("Disconnected", "info")
        
    def serial_worker(self):
        """Background thread for handling serial communication"""
        while self.running and self.connected:
            try:
                # Check for incoming encoder data
                if self.ser.in_waiting >= 9:
                    data = self.ser.read(9)
                    if len(data) == 9 and data[0] == 0xFF:
                        # Parse encoder data
                        enc0 = int.from_bytes(data[1:3], byteorder='big', signed=True)
                        rpm0 = int.from_bytes(data[3:5], byteorder='big', signed=True)
                        enc1 = int.from_bytes(data[5:7], byteorder='big', signed=True)
                        rpm1 = int.from_bytes(data[7:9], byteorder='big', signed=True)
                        
                        # Update encoder data
                        enc_data = {
                            'motor0': {'position': enc0, 'rpm': rpm0},
                            'motor1': {'position': enc1, 'rpm': rpm1}
                        }
                        self.encoder_queue.put(enc_data)
                        
                time.sleep(0.01)  # Small delay to prevent busy waiting
                
            except Exception as e:
                if self.running:
                    self.log_message(f"Serial error: {e}", "error")
                break
                
    def send_command(self, cmd_bytes, expected_ack=None):
        """Send command and wait for acknowledgment"""
        if not self.connected:
            self.log_message("Not connected!", "error")
            return False
            
        try:
            # Clear input buffer
            self.ser.reset_input_buffer()
            
            # Send command
            for byte in cmd_bytes:
                self.ser.write(bytes([byte]))
                time.sleep(0.01)
                
            # Wait for acknowledgment if expected
            if expected_ack:
                time.sleep(0.05)
                resp = self.ser.read(1)
                if len(resp) == 1 and resp[0] == expected_ack:
                    return True
                else:
                    resp_str = f"{resp[0]:02X}" if resp else 'None'
                    self.log_message(f"Wrong ack: expected {expected_ack:02X}, got {resp_str}", "warning")
                    return False
            return True
            
        except Exception as e:
            self.log_message(f"Command failed: {e}", "error")
            return False
            
    def send_pwm(self):
        try:
            pwm0 = int(self.motor0_var.get())
            pwm1 = int(self.motor1_var.get())
            
            # Clamp values
            pwm0 = max(-self.current_ccr, min(self.current_ccr, pwm0))
            pwm1 = max(-self.current_ccr, min(self.current_ccr, pwm1))
            
            # Send Motor 0
            pwm0_bytes = pwm0.to_bytes(2, byteorder='big', signed=True)
            cmd0 = [0xAA, pwm0_bytes[0], pwm0_bytes[1]]
            success0 = self.send_command(cmd0, 0xAA)
            
            time.sleep(0.1)
            
            # Send Motor 1
            pwm1_bytes = pwm1.to_bytes(2, byteorder='big', signed=True)
            cmd1 = [0xBB, pwm1_bytes[0], pwm1_bytes[1]]
            success1 = self.send_command(cmd1, 0xBB)
            
            if success0 and success1:
                self.log_message(f"PWM sent: Motor0={pwm0}, Motor1={pwm1}", "success")
            else:
                self.log_message("PWM command failed", "error")
                
        except ValueError:
            self.log_message("Invalid PWM values", "error")
            
    def stop_motors(self):
        self.motor0_var.set("0")
        self.motor1_var.set("0")
        self.send_pwm()
        
    def send_config(self, config_type):
        try:
            if config_type == "psc":
                value = int(self.psc_var.get())
                header = 0xCC
                self.current_psc = value
            elif config_type == "ccr":
                value = int(self.ccr_var.get())
                header = 0xDD
                self.current_ccr = value
            elif config_type == "ppr":
                value = int(self.ppr_var.get())
                header = 0x11
                self.current_ppr = value
            else:
                return
                
            value_bytes = value.to_bytes(2, byteorder='big', signed=False)
            cmd = [header, value_bytes[0], value_bytes[1]]
            
            if self.send_command(cmd, header):
                self.log_message(f"{config_type.upper()} set to {value}", "success")
            else:
                self.log_message(f"{config_type.upper()} command failed", "error")
                
        except ValueError:
            self.log_message(f"Invalid {config_type.upper()} value", "error")
            
    def query_settings(self):
        """Query current FPGA settings - SIMPLIFIED for debugging"""
        if self.send_command([0xEE], 0xEE):
            self.log_message(f"Query successful - using current values: PSC={self.current_psc}, CCR={self.current_ccr}, PPR={self.current_ppr}", "success")
        else:
            self.log_message("Query failed", "error")
    
    def send_pid_mode(self, motor_id):
        """Send PID mode selection command"""
        if not self.connected:
            self.log_message("Not connected to FPGA", "error")
            return
            
        try:
            mode_var = self.motor0_mode if motor_id == 0 else self.motor1_mode
            mode_value = 1 if mode_var.get() == "PID" else 0
            
            cmd = [0x22, motor_id, mode_value]
            
            if self.send_command(cmd, 0x22):
                motor_name = f"Motor {motor_id}"
                mode_name = "PID" if mode_value else "PWM"
                self.log_message(f"{motor_name} mode set to {mode_name}", "success")
                # Update display immediately
                self.update_mode_display()
            else:
                self.log_message(f"Motor {motor_id} mode command failed", "error")
                
        except Exception as e:
            self.log_message(f"Error setting motor mode: {e}", "error")
    
    def send_pid_param(self, motor_id, param_type):
        """Send PID parameter command"""
        if not self.connected:
            self.log_message("Not connected to FPGA", "error")
            return
            
        try:
            # Get the appropriate variable based on motor and parameter
            if motor_id == 0:
                param_vars = {
                    'setpoint': self.motor0_setpoint,
                    'kp': self.motor0_kp,
                    'ki': self.motor0_ki,
                    'kd': self.motor0_kd
                }
            else:
                param_vars = {
                    'setpoint': self.motor1_setpoint,
                    'kp': self.motor1_kp,
                    'ki': self.motor1_ki,
                    'kd': self.motor1_kd
                }
            
            value = int(param_vars[param_type].get())
            
            # Validate parameter ranges
            if param_type == 'setpoint':
                # Setpoint can be negative for reverse direction, but limit range
                if value < -3000 or value > 3000:
                    self.log_message(f"Setpoint out of range (-3000 to +3000 RPM): {value}", "error")
                    return
            else:
                # PID gains should be positive
                if value < 0:
                    self.log_message(f"{param_type.upper()} must be positive: {value}", "error")
                    return
                if value > 65535:
                    self.log_message(f"{param_type.upper()} too large (max 65535): {value}", "error")
                    return
            
            # Command headers for different parameters
            headers = {
                'setpoint': 0x33,
                'kp': 0x44,
                'ki': 0x55,
                'kd': 0x66
            }
            
            header = headers[param_type]
            
            # PID parameter commands use 4 bytes: [header][motor_id][data_high][data_low]
            if param_type == 'setpoint':
                # For setpoint, allow negative values (signed) for reverse direction
                value_bytes = value.to_bytes(2, byteorder='big', signed=True)
                cmd = [header, motor_id, value_bytes[0], value_bytes[1]]
            else:
                # For Kp, Ki, Kd: should be positive values (unsigned)
                value_bytes = value.to_bytes(2, byteorder='big', signed=False)
                cmd = [header, motor_id, value_bytes[0], value_bytes[1]]
            
            if self.send_command(cmd, header):
                self.log_message(f"Motor {motor_id} {param_type.upper()} set to {value}", "success")
                # Update display if setpoint changed
                if param_type == 'setpoint':
                    self.update_mode_display()
            else:
                self.log_message(f"Motor {motor_id} {param_type.upper()} command failed", "error")
                
        except ValueError:
            self.log_message(f"Invalid {param_type.upper()} value", "error")
        except Exception as e:
            self.log_message(f"Error setting {param_type}: {e}", "error")
            
    def clear_log(self):
        self.log_text.delete(1.0, tk.END)
        
    def on_closing(self):
        self.running = False
        if self.connected:
            self.disconnect()
        self.root.destroy()
        
    def update_mode_display(self):
        """Update mode display labels and setpoint visibility"""
        # Motor 0 mode display
        mode0 = self.motor0_mode.get()
        self.motor0_mode_label.config(text=f"Mode: {mode0}")
        
        if mode0 == "PID":
            setpoint0 = self.motor0_setpoint.get()
            self.motor0_setpoint_label.config(text=f"Setpoint: {setpoint0} RPM")
            self.motor0_mode_label.config(foreground="red")  # Red for PID mode
            self.motor0_error_label.grid()  # Show error label
        else:
            self.motor0_setpoint_label.config(text="Setpoint: -")
            self.motor0_mode_label.config(foreground="blue")  # Blue for PWM mode
            self.motor0_error_label.config(text="Error: -")
            
        # Motor 1 mode display
        mode1 = self.motor1_mode.get()
        self.motor1_mode_label.config(text=f"Mode: {mode1}")
        
        if mode1 == "PID":
            setpoint1 = self.motor1_setpoint.get()
            self.motor1_setpoint_label.config(text=f"Setpoint: {setpoint1} RPM")
            self.motor1_mode_label.config(foreground="red")  # Red for PID mode
            self.motor1_error_label.grid()  # Show error label
        else:
            self.motor1_setpoint_label.config(text="Setpoint: -")
            self.motor1_mode_label.config(foreground="blue")  # Blue for PWM mode
            self.motor1_error_label.config(text="Error: -")

if __name__ == "__main__":
    root = tk.Tk()
    app = MotorControlGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()
