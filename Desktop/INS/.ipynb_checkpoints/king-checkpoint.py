import serial
import csv
import time

# Open serial port (adjust 'COM13' and baud rate as necessary)
ser = serial.Serial('COM13', 115200, timeout=1)
ser.flush()

# Open CSV file for writing
with open('sensor_data.csv', 'w', newline='') as csvfile:
    csv_writer = csv.writer(csvfile)
    # Write the header
    csv_writer.writerow([ "AccX", "AccY", "AccZ" ,"GyrX", "GyrY", "GyrZ"])

    print("Starting data collection...")

    try:
        while True:
            if ser.in_waiting > 0:
                # Read a line from the serial port
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                # Write the data to the CSV file
                if line:
                    csv_writer.writerow(line.split(','))
                    print(f"Data saved: {line}")
    except KeyboardInterrupt:
        # Handle script interruption (e.g., Ctrl+C)
        print("Data collection stopped.")
    finally:
        # Close the serial port
        ser.close()
