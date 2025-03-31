import serial
import csv
import time

# Open the serial port
ser = serial.Serial('COM3', 9600, timeout=1)  # Add a timeout to avoid hanging

# Open the CSV file in append mode
with open('sensor_data.csv', mode='a', newline='') as file:
    writer = csv.writer(file)

    # Add a header if the file is empty
    if file.tell() == 0:
        writer.writerow(['Temperature (°C)', 'Humidity (%)', 'Paper Sensor Value'])

    print("Started saving data...")

    try:
        while True:
            # Read a line of data from the Arduino
            line = ser.readline().decode('utf-8').strip()
            print(f"Raw data: {line}")  # Debugging raw data

            if line:
                # Split the line into values
                data = line.split(',')

                if len(data) == 3:  # Ensure three values are received
                    temperature = data[0]
                    humidity = data[1]
                    PaperSensorValue = data[2]

                    # Write the data to the CSV file
                    writer.writerow([temperature, humidity, PaperSensorValue])
                    file.flush()  # Ensure data is written to the file immediately

                    # Print the data to console
                    print(f"{temperature}C, {humidity}%, {PaperSensorValue}")
                else:
                    print("Incomplete data received, skipping entry.")

                # Wait for a short time before reading again
                time.sleep(1)
    except KeyboardInterrupt:
        print("Data saving stopped.")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Close the serial port
        ser.close()
