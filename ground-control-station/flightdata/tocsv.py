import struct
import csv
import argparse

def parse_flight_data(binary_file, csv_file):
    MSG_START_BYTE = 0xFA  # Assuming a constant start byte
    HEADER_SIZE = 3  # Start byte (1) + Type byte (1) + Size byte (1)
    FLOAT_SIZE = 4  # Each float is 4 bytes
    FIELD_NAMES = [
        "mass", "time", "ax", "ay", "az", "ax_local", "ay_local", "az_local",
        "pitch", "roll", "yaw", "vx", "vy", "vz", "vx_local", "vy_local", "vz_local",
        "x", "y", "z", "qw", "qx", "qy", "qz", "apogee", "baro_altitude",
        "altitude", "baro_pressure", "baro_temperature", "air_pressure",
        "air_density", "air_temperature", "drag_coefficient"
    ]
    NUM_FLOATS = len(FIELD_NAMES)
    MESSAGE_SIZE = NUM_FLOATS * FLOAT_SIZE  # Total message data size
    
    with open(binary_file, "rb") as bin_file, open(csv_file, "w", newline="") as csv_out:
        writer = csv.writer(csv_out)
        writer.writerow(["msg_type"] + FIELD_NAMES)  # CSV header
        
        while True:
            header = bin_file.read(HEADER_SIZE)
            if len(header) < HEADER_SIZE:
                break  # End of file
            
            start_byte, msg_type, msg_size = struct.unpack("BBB", header)
            if start_byte != MSG_START_BYTE:
                print("womp womp\n")
                bin_file.seek(bin_file.tell()-2)
                continue  # Skip invalid messages
            
            data_bytes = bin_file.read(msg_size)
            if len(data_bytes) < msg_size:
                break  # Incomplete message
            
            values = struct.unpack("<" + "f" * (int)((msg_size)/4), data_bytes)  # Little-endian float unpacking
            writer.writerow([msg_type] + list(values))
    
    print(f"CSV file '{csv_file}' successfully created.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convert binary flight data to CSV.")
    parser.add_argument("input_file", help="Path to the binary flight data file.")
    parser.add_argument("output_file", help="Path to the output CSV file.")

    args = parser.parse_args()
    parse_flight_data(args.input_file, args.output_file)