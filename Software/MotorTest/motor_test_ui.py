import customtkinter as ctk
import serial
import serial.tools.list_ports

class MotorApp(ctk.CTk):
    def __init__(self):
        super().__init__()
        self.title("RAMPS 6-Axis Control")
        self.geometry("400x550")  # Extended vertical size to fit 6 axes
        self.arduino = None
        self.create_widgets()

    def create_widgets(self):
        # Port Selection Panel
        self.conn_frame = ctk.CTkFrame(self)
        self.conn_frame.pack(pady=10, padx=10, fill="x")

        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.port_combo = ctk.CTkComboBox(self.conn_frame, values=ports if ports else ["No Port"])
        self.port_combo.pack(side="left", padx=5)

        self.btn_conn = ctk.CTkButton(self.conn_frame, text="Connect", fg_color="green", command=self.toggle)
        self.btn_conn.pack(side="right", padx=5)

        # --------------------------------------------------------
        # ORIGINAL STEP/DIR MOTORS
        # --------------------------------------------------------
        self.add_axis("X Axis", "XL", "XR", "XS")
        self.add_axis("Y Axis", "YL", "YR", "YS")
        self.add_axis("Z Axis", "ZL", "ZR", "ZS")

        # Leave a space like a separator line
        spacer = ctk.CTkLabel(self, text="--- ULN2003 Motors ---", text_color="gray")
        spacer.pack(pady=5)

        # --------------------------------------------------------
        # NEWLY ADDED ULN MOTORS
        # --------------------------------------------------------
        self.add_axis("U Axis", "UL", "UR", "US")
        self.add_axis("V Axis", "VL", "VR", "VS")
        self.add_axis("W Axis", "WL", "WR", "WS")

    def add_axis(self, name, l, r, s):
        f = ctk.CTkFrame(self)
        f.pack(pady=5, padx=10, fill="x")
        ctk.CTkLabel(f, text=name, width=60).pack(side="left", padx=10)

        btn_l = ctk.CTkButton(f, text="◀", width=50)
        btn_l.bind("<ButtonPress-1>", lambda e: self.send(l))
        btn_l.bind("<ButtonRelease-1>", lambda e: self.send(s))
        btn_l.pack(side="left", padx=5)

        btn_r = ctk.CTkButton(f, text="▶", width=50)
        btn_r.bind("<ButtonPress-1>", lambda e: self.send(r))
        btn_r.bind("<ButtonRelease-1>", lambda e: self.send(s))
        btn_r.pack(side="left", padx=5)

    def toggle(self):
        if not self.arduino:
            try:
                self.arduino = serial.Serial(self.port_combo.get(), 9600, timeout=1)
                self.btn_conn.configure(text="Connected (Disconnect)", fg_color="red")
            except Exception as e:
                print(f"Error: {e}")
        else:
            self.arduino.close()
            self.arduino = None
            self.btn_conn.configure(text="Connect", fg_color="green")

    def send(self, cmd):
        if self.arduino and self.arduino.is_open:
            self.arduino.write((cmd + "\n").encode())

if __name__ == "__main__":
    MotorApp().mainloop()
