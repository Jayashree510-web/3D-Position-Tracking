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
    csv_writer.writerow(["Time(ms)", "AccX", "AccY", "AccZ", "GyrX", "GyrY", "GyrZ"])

    print("Starting data collection...")

    try:
        while True:
            if ser.in_waiting > 0:
                # Read a line from the serial port
                line = ser.readline().decode('utf-8', errors='ignore').strip()

                # Write the data to the CSV file
                if line:
                    data = line.split(',')
                    
                    # Ensure we have exactly 7 values (time + 6 sensor readings)
                    if len(data) == 7:
                        csv_writer.writerow(data)
                        csvfile.flush()  # Ensure real-time saving
                        print(f"Data saved: {line}")
                    
                time.sleep(0.01)  # Prevent excessive CPU usage

    except KeyboardInterrupt:
        print("\nData collection stopped by user.")

    finally:
        ser.close()
        print("Serial port closed.")
