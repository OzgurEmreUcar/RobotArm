#!/usr/bin/env python3
import tkinter as tk
from tkinter import ttk
import serial
import serial.tools.list_ports
import time
import threading
import json
import os
import rclpy
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter as ParameterMsg
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.msg import ParameterType

# --- AYARLAR ---
BAUD_RATE = 115200
SEND_INTERVAL_MS = 25         
DURATION_MS = 30              

CONFIG_FILE = os.path.join(os.path.dirname(__file__), 'serial_config.json')

def load_serial_port():
    if os.path.exists(CONFIG_FILE):
        try:
            with open(CONFIG_FILE, 'r') as f:
                data = json.load(f)
                return data.get('serial_port', '/dev/ttyUSB0')
        except Exception:
            pass
    return '/dev/ttyUSB0'

def save_serial_port(port):
    try:
        with open(CONFIG_FILE, 'w') as f:
            json.dump({'serial_port': port}, f)
    except Exception as e:
        print(f"Ayar kaydedilemedi: {e}")

class RobotJogApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Robot Manuel Hizalama (Jog) Paneli")
        self.root.geometry("450x700")
        
        self.is_pressing = False
        self.active_joint = 0
        self.active_direction = 0  
        
        # ROS 2 node setup for remote parameter setting
        if not rclpy.ok():
            rclpy.init(args=None)
        self.ros_node = rclpy.create_node('robot_jog_gui_node')
        
        # NOTE: robot_arm.launch.py overrides the node name to "trajectory_driver_node"
        self.param_client = self.ros_node.create_client(SetParameters, '/trajectory_driver_node/set_parameters')
        
        self.ros_thread = threading.Thread(target=self._spin_ros, daemon=True)
        self.ros_thread.start()
        
        # Robotun genel güç durumu (True: Kilitli/Enerjili, False: Serbest)
        self.robot_is_enabled = True

        self.nema17_speed = tk.DoubleVar(value=120.0) 
        self.byj48_speed = tk.DoubleVar(value=15.0)   

        self.ser = None

        # --- PORT SEÇİMİ ARAYÜZÜ ---
        port_frame = tk.Frame(root)
        port_frame.pack(pady=5, fill="x", padx=20)
        
        tk.Label(port_frame, text="Seri Port:", font=("Arial", 10)).pack(side=tk.LEFT)
        
        self.port_var = tk.StringVar()
        self.port_var.set(load_serial_port())
        
        self.port_cb = ttk.Combobox(port_frame, textvariable=self.port_var, width=15)
        self.port_cb.pack(side=tk.LEFT, padx=5)
        
        self.btn_refresh = tk.Button(port_frame, text="Yenile", command=self.refresh_ports)
        self.btn_refresh.pack(side=tk.LEFT, padx=5)
        
        self.btn_connect = tk.Button(port_frame, text="Bağlan", command=self.connect_serial_thread, bg="#add8e6")
        self.btn_connect.pack(side=tk.LEFT, padx=5)
        
        self.lbl_conn_status = tk.Label(port_frame, text="Bağlı Değil", fg="red", font=("Arial", 9, "bold"))
        self.lbl_conn_status.pack(side=tk.LEFT, padx=10)
        
        self.refresh_ports()

        lbl = tk.Label(root, text="Eksenleri Hizalamak İçin Butonlara Basılı Tutun", font=("Arial", 11, "bold"))
        lbl.pack(pady=10)
        
        # --- BUTONLARIN OLUŞTURULMASI ---
        for j in range(1, 7):
            frame = tk.Frame(root)
            frame.pack(pady=5)
            
            lbl_joint = tk.Label(frame, text=f"Eksen {j}: ", width=8, anchor="w", font=("Arial", 10))
            lbl_joint.pack(side=tk.LEFT)
            
            btn_minus = tk.Button(frame, text=" < (-)", width=10, bg="#ff9999", activebackground="#ff6666")
            btn_minus.pack(side=tk.LEFT, padx=5)
            btn_minus.bind("<ButtonPress-1>", lambda event, joint=j: self.start_jog(joint, -1))
            btn_minus.bind("<ButtonRelease-1>", self.stop_jog)
            
            btn_plus = tk.Button(frame, text=" > (+)", width=10, bg="#99ff99", activebackground="#66ff66")
            btn_plus.pack(side=tk.LEFT, padx=5)
            btn_plus.bind("<ButtonPress-1>", lambda event, joint=j: self.start_jog(joint, 1))
            btn_plus.bind("<ButtonRelease-1>", self.stop_jog)

        # --- ARAYÜZ HIZ AYARLARI (SLIDERS) ---
        slider_frame = tk.LabelFrame(root, text=" Canlı Hız Ayarları (Adım Boyutu) ", font=("Arial", 10, "bold"), padx=10, pady=10)
        slider_frame.pack(pady=10, fill="x", padx=20)

        lbl_nema = tk.Label(slider_frame, text="Eksen 1-2-3 (NEMA 17):", font=("Arial", 9))
        lbl_nema.pack(anchor="w")
        scale_nema = tk.Scale(slider_frame, from_=10, to=300, orient=tk.HORIZONTAL, variable=self.nema17_speed)
        scale_nema.pack(fill="x", pady=(0, 10))

        lbl_byj = tk.Label(slider_frame, text="Eksen 4-5-6 (28BYJ-48):", font=("Arial", 9))
        lbl_byj.pack(anchor="w")
        scale_byj = tk.Scale(slider_frame, from_=1, to=100, orient=tk.HORIZONTAL, variable=self.byj48_speed)
        scale_byj.pack(fill="x")

        # --- ROBOT İÇİN ENERJİ KESME / TORK BUTONU ---
        power_frame = tk.LabelFrame(root, text=" Robot Genel Güç Yönetimi ", font=("Arial", 10, "bold"), padx=10, pady=10)
        power_frame.pack(pady=10, fill="x", padx=20)

        self.btn_toggle_power = tk.Button(power_frame, text="TÜM ENERJİYİ KES (Robotu Serbest Bırak)", 
                                          font=("Arial", 10, "bold"), bg="#ff3333", fg="white", 
                                          command=self.toggle_robot_power, height=2)
        self.btn_toggle_power.pack(fill="x")

        # --- BİLGİ ALANI ---
        info_frame = tk.Frame(root)
        info_frame.pack(pady=5)
        lbl_info = tk.Label(info_frame, text="Sıfırlama İşlemi:\nRobotu başlangıç (0) konumuna getirdiğinizde\nArduino üzerindeki RESET butonuna basın.", 
                            font=("Arial", 9, "italic"), fg="blue", justify=tk.CENTER)
        lbl_info.pack()

        # Otomatik olarak seçili porta bağlanmaya çalış
        self.connect_serial_thread()

    def refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        if not ports:
            ports = ["/dev/ttyUSB0", "/dev/ttyACM0"] # Default fallbacks
        if self.port_var.get() not in ports:
            ports.append(self.port_var.get())
        self.port_cb['values'] = ports

    def _spin_ros(self):
        while rclpy.ok():
            rclpy.spin_once(self.ros_node, timeout_sec=0.1)

    def update_remote_parameter(self, port):
        self.ros_node.get_logger().info(f"[JOG GUI] Parametre guncelleme istegi baslatiliyor... Hedef port: {port}")
        if self.param_client.wait_for_service(timeout_sec=2.0):
            req = SetParameters.Request()
            val = ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=port)
            req.parameters = [ParameterMsg(name='serial_port', value=val)]
            future = self.param_client.call_async(req)
            
            def future_cb(f):
                try:
                    res = f.result()
                    success = res.results[0].successful
                    reason = res.results[0].reason
                    self.ros_node.get_logger().info(f"[JOG GUI] Parametre guncelleme sonucu: Basarili={success}, Sebep='{reason}'")
                except Exception as e:
                    self.ros_node.get_logger().error(f"[JOG GUI] Parametre guncelleme sirasinda hata olustu: {e}")
                    
            future.add_done_callback(future_cb)
            self.ros_node.get_logger().info(f"[JOG GUI] trajectory_driver_node dugumune '{port}' istegi gonderildi, yanit bekleniyor...")
        else:
            self.ros_node.get_logger().error("[JOG GUI] HATA: /trajectory_driver_node/set_parameters servisi 2 saniye icinde bulunamadi! Dugum baslamamis olabilir.")

    def connect_serial_thread(self):
        threading.Thread(target=self.connect_serial, daemon=True).start()

    def connect_serial(self):
        if self.ser:
            try:
                self.ser.close()
            except:
                pass
            self.ser = None
            
        port = self.port_var.get()
        save_serial_port(port) # Seçimi kaydet
        
        # Ana node'un (trajectory_driver) portunu uzaktan güncelle
        self.update_remote_parameter(port)
        
        self.lbl_conn_status.config(text="Bağlanıyor...", fg="orange")
        
        try:
            self.ser = serial.Serial(port, BAUD_RATE, timeout=0.05)
            time.sleep(2)  # Arduino'nun bootloader'ı varsa reset'ten kurtulmasını bekle
            self.lbl_conn_status.config(text="Bağlı", fg="green")
            print(f"-> {port} portuna bağlantı başarılı.")
        except Exception as e:
            self.lbl_conn_status.config(text="Hata!", fg="red")
            print(f"-> Port açılamadı: {e}")
            self.ser = None

    def toggle_robot_power(self):
        if not self.ser:
            print("Seri port bağlı değil!")
            return

        if self.robot_is_enabled:
            try:
                self.ser.write(b"<FREEZE>\n")
                self.robot_is_enabled = False
                self.btn_toggle_power.config(text="ROBOTA ENERJİ VER (Tüm Motorları Kilitle)", bg="#228B22", fg="white")
                print("-> Tüm motorların enerjisi kesildi. Robot tamamen serbest.")
            except Exception as e:
                print(f"Hata: {e}")
        else:
            try:
                self.ser.write(b"<ENGAGE>\n")
                self.robot_is_enabled = True
                self.btn_toggle_power.config(text="TÜM ENERJİYİ KES (Robotu Serbest Bırak)", bg="#ff3333", fg="white")
                print("-> Tüm motorlara enerji verildi. Robot kilitlendi.")
            except Exception as e:
                print(f"Hata: {e}")

    def start_jog(self, joint, direction):
        if not self.ser:
            print("Seri port bağlı değil! Lütfen önce bağlanın.")
            return

        # 🔴 OTOMATİK ENERJİLENDİRME MANTIĞI:
        # Eğer robot serbest bırakılmışsa ve bir hareket komutu geldiyse butona basılmasını bekleme!
        if not self.robot_is_enabled:
            print("-> Hareket komutu algılandı! Robot otomatik olarak enerjilendiriliyor...")
            try:
                self.ser.write(b"<ENGAGE>\n")
                self.robot_is_enabled = True
                self.btn_toggle_power.config(text="TÜM ENERJİYİ KES (Robotu Serbest Bırak)", bg="#ff3333", fg="white")
                time.sleep(0.05) # Sürücülerin kendine gelmesi ve motorların kilitlenmesi için 50ms minik bir es veriyoruz
            except Exception as e:
                print(f"Otomatik enerjilendirme hatası: {e}")

        if not self.is_pressing:
            self.is_pressing = True
            self.active_joint = joint
            self.active_direction = direction
            threading.Thread(target=self.jog_loop, daemon=True).start()

    def stop_jog(self, event):
        self.is_pressing = False

    def jog_loop(self):
        while self.is_pressing:
            move = [0, 0, 0, 0, 0, 0]
            
            if self.active_joint <= 3:
                step_size = int(self.nema17_speed.get())
            else:
                step_size = int(self.byj48_speed.get())
                
            move[self.active_joint - 1] = step_size * self.active_direction
            
            cmd = f"<{move[0]},{move[1]},{move[2]},{move[3]},{move[4]},{move[5]},{DURATION_MS}>\n"
            
            if self.ser:
                try:
                    self.ser.write(cmd.encode('utf-8'))
                except Exception as e:
                    print(f"Seri port hatası: {e}")
            
            time.sleep(SEND_INTERVAL_MS / 1000.0)

def main():
    if not rclpy.ok():
        rclpy.init(args=None)
    root = tk.Tk()
    app = RobotJogApp(root)
    root.mainloop()
    if app.ser:
        app.ser.close()
    
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == "__main__":
    main()