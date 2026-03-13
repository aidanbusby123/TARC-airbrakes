import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import json
import serial
import serial.tools.list_ports
import struct

# --- MATCH THESE TO YOUR ARDUINO CODE ---
MSG_START_BYTE = 0xAA  
MSG_TYPE_CONFIG = 0x02 

class ConfigApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Teensy Config Utility")
        
        self.config_data = {
            "ref_area": 0.00343, "drag_coefficient": 0.39, "brake_coefficient_full_deploy": 1.0, "target_apogee": 228.0,
            "trigger_acceleration": 10.0, "max_time": 40.0, "brake_retracted": 35.0,
            "brake_deployed": 80.0, "kp": 0.675, "ki": 0.3, "kd": 0.05, "mass": 0.5, "use_ekf": 0.0
        }
        
        self.entries = {}
        self.create_widgets()
        self.refresh_ports() # Initial port scan

    def create_widgets(self):
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.pack(fill="both", expand=True)

        # --- Parameters ---
        config_frame = ttk.LabelFrame(main_frame, text=" Configuration Parameters ", padding=10)
        config_frame.pack(padx=5, pady=5, fill="x")

        for i, (key, value) in enumerate(self.config_data.items()):
            ttk.Label(config_frame, text=key).grid(row=i, column=0, sticky="w", pady=2)
            ent = ttk.Entry(config_frame)
            ent.insert(0, str(value))
            ent.grid(row=i, column=1, padx=10, pady=2, sticky="e")
            self.entries[key] = ent

        # --- Serial Connection ---
        serial_frame = ttk.LabelFrame(main_frame, text=" Serial Connection ", padding=10)
        serial_frame.pack(padx=5, pady=5, fill="x")

        self.port_var = tk.StringVar()
        self.port_menu = ttk.Combobox(serial_frame, textvariable=self.port_var, state="readonly")
        self.port_menu.pack(side="left", padx=5)
        
        ttk.Button(serial_frame, text="Refresh Ports", command=self.refresh_ports).pack(side="left", padx=5)

        # --- Actions ---
        btn_frame = ttk.Frame(main_frame, padding=10)
        btn_frame.pack(fill="x")

        ttk.Button(btn_frame, text="Load File", command=self.load_from_file).pack(side="left", padx=5)
        ttk.Button(btn_frame, text="Save File", command=self.save_to_file).pack(side="left", padx=5)
        ttk.Button(btn_frame, text="Upload to Arduino", command=self.upload_serial).pack(side="right", padx=5)

    def refresh_ports(self):
        """Updates the list of available COM ports."""
        ports = [p.device for p in serial.tools.list_ports.comports()]
        if ports:
            self.port_menu['values'] = ports
            self.port_menu.current(0)
        else:
            self.port_menu['values'] = ["No Ports Found"]
            self.port_menu.current(0)

    def get_dict_from_ui(self):
        current_values = {}
        for key, entry in self.entries.items():
            try:
                current_values[key] = float(entry.get())
            except ValueError:
                messagebox.showerror("Error", f"Invalid input for {key}")
                return None
        return current_values

    def load_from_file(self):
        path = filedialog.askopenfilename(filetypes=[("JSON files", "*.json")])
        if path:
            try:
                with open(path, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                    for key, val in data.items():
                        if key in self.entries:
                            self.entries[key].delete(0, tk.END)
                            self.entries[key].insert(0, str(val))
            except Exception as e:
                messagebox.showerror("Load Error", str(e))

    def save_to_file(self):
        """Saves as straight text JSON (Pretty print)."""
        data_dict = self.get_dict_from_ui()
        if data_dict:
            path = filedialog.asksaveasfilename(defaultextension=".json")
            if path:
                with open(path, 'w', encoding='utf-8') as f:
                    json.dump(data_dict, f, indent=4)
                messagebox.showinfo("Success", "Saved as human-readable JSON.")

    def upload_serial(self):
        """Uploads as compact binary packet with Null Terminator."""
        data_dict = self.get_dict_from_ui()
        port = self.port_var.get()
        
        if not data_dict or not port or port == "No Ports Found":
            messagebox.showwarning("Warning", "Please select a valid Serial Port.")
            return

        # Compact JSON (no newlines/spaces) to keep packet size small
        json_compact = json.dumps(data_dict, separators=(',', ':'))
        payload_bytes = json_compact.encode('utf-8') + b'\x00'
        data_size = len(payload_bytes)

        if data_size > 255:
            messagebox.showerror("Error", "Packet size exceeds uint8_t limit (255).")
            return

        try:
            # Struct Format: B=UnsignedChar(1 byte), s=Bytes
            packet = struct.pack(f'BBB{data_size}s', 
                                 MSG_START_BYTE, 
                                 MSG_TYPE_CONFIG, 
                                 data_size, 
                                 payload_bytes)

            # Teensy usually works at any baud rate, but 115200 is standard
            with serial.Serial(port, 115200, timeout=1) as ser:
                ser.write(packet)
                messagebox.showinfo("Success", f"Sent {data_size} bytes to {port}")
        except Exception as e:
            messagebox.showerror("Serial Error", f"Could not open {port}:\n{e}")

if __name__ == "__main__":
    root = tk.Tk()
    app = ConfigApp(root)
    root.mainloop()