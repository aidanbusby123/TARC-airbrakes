import struct
import csv
import tkinter as tk
from tkinter import filedialog, messagebox

class FlightDataConverterGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Flight Data to CSV Converter")
        self.root.geometry("500x200")

        # --- Variables to store paths ---
        self.input_path = tk.StringVar()
        self.output_path = tk.StringVar()

        # --- UI Layout ---
        tk.Label(root, text="Binary Input File:").grid(row=0, column=0, padx=10, pady=10, sticky="w")
        tk.Entry(root, textvariable=self.input_path, width=40).grid(row=0, column=1, padx=5)
        tk.Button(root, text="Browse", command=self.browse_input).grid(row=0, column=2, padx=5)

        tk.Label(root, text="CSV Output File:").grid(row=1, column=0, padx=10, pady=10, sticky="w")
        tk.Entry(root, textvariable=self.output_path, width=40).grid(row=1, column=1, padx=5)
        tk.Button(root, text="Browse", command=self.browse_output).grid(row=1, column=2, padx=5)

        tk.Button(root, text="Convert Data", command=self.run_conversion, 
                  bg="#4CAF50", fg="white", font=('Arial', 10, 'bold')).grid(row=2, column=1, pady=20)

    def browse_input(self):
        file = filedialog.askopenfilename(title="Select Binary Flight Data", 
                                          filetypes=[("Binary files", "*.dat"), ("All files", "*.*")])
        if file:
            self.input_path.set(file)

    def browse_output(self):
        file = filedialog.asksaveasfilename(title="Select Output CSV Location", 
                                            defaultextension=".csv",
                                            filetypes=[("CSV files", "*.csv")])
        if file:
            self.output_path.set(file)

    def run_conversion(self):
        in_file = self.input_path.get()
        out_file = self.output_path.get()

        if not in_file or not out_file:
            messagebox.showwarning("Input Error", "Please select both input and output files.")
            return

        try:
            self.parse_flight_data(in_file, out_file)
            messagebox.showinfo("Success", f"Conversion complete!\nSaved to: {out_file}")
        except Exception as e:
            messagebox.showerror("Error", f"An error occurred:\n{str(e)}")

    def parse_flight_data(self, binary_file, csv_file):
        MSG_START_BYTE = 0xFA
        HEADER_SIZE = 3
        FIELD_NAMES = [
            "mass", "time", "ax", "ay", "az", "ax_local", "ay_local", "az_local",
            "pitch", "roll", "yaw", "vx", "vy", "vz", "vx_local", "vy_local", "vz_local",
            "x", "y", "z", "qw", "qx", "qy", "qz", "apogee", "baro_altitude",
            "altitude", "baro_pressure", "baro_temperature", 
            "air_density", "drag_coefficient", "p", "i", "d", "pid", "brake_target_deployment"
        ]
        
        with open(binary_file, "rb") as bin_file, open(csv_file, "w", newline="") as csv_out:
            writer = csv.writer(csv_out)
            writer.writerow(["msg_type"] + FIELD_NAMES)
            
            while True:
                header = bin_file.read(HEADER_SIZE)
                if len(header) < HEADER_SIZE:
                    break
                
                start_byte, msg_type, msg_size = struct.unpack("BBB", header)
                if start_byte != MSG_START_BYTE:
                    # Slide the window by 1 byte if we lose sync
                    bin_file.seek(bin_file.tell() - (HEADER_SIZE - 1))
                    continue
                
                data_bytes = bin_file.read(msg_size)
                if len(data_bytes) < msg_size:
                    break
                
                # Dynamic float unpacking based on actual message size
                num_floats = msg_size // 4
                values = struct.unpack("<" + "f" * num_floats, data_bytes)
                writer.writerow([msg_type] + list(values))

if __name__ == "__main__":
    root = tk.Tk()
    app = FlightDataConverterGUI(root)
    root.mainloop()