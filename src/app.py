import serial
import pymongo
import time
from datetime import datetime

# Setup UART
ser = serial.Serial('COM3', 115200)  # Adjust to your serial port and baud rate
ser2 = serial.Serial('COM17',115200)
# Setup MongoDB Atlas
client = pymongo.MongoClient("mongodb+srv://fsiddiqui107:gc79mKY4g6hGrbVL@ssnscluster.fsy0znp.mongodb.net/?retryWrites=true&w=majority&appName=SSNSCluster")
db = client['testing']
collection = db['testing']

while True:
    try:
        if ser.in_waiting > 0 and ser2.in_waiting > 0 :
            data = ser.readline().decode('utf-8').strip()
            data2 = ser2.readline().decode('utf-8').strip()
            print("Gooaaaaal",data2.split(':')[1].split()[0])
            print(f"Received: {data}")

            # Parse the data
            parts = data.split(',')
            
            # Extract temperature
            temperature = float(parts[0].split(':')[1].strip().split()[0])
            
            # Extract humidity
            humidity = float(parts[1].split(':')[1].strip().split('%')[0])
            
            # Extract pressure
            pressure = float(parts[2].split(':')[1].strip().split()[0])

            # Fixed value for particulate matter
            particulate_matter = data2.split(':')[1].split()[0]

            # Get the current time and date
            current_time = datetime.now().strftime("%H:%M:%S")
            current_date = datetime.now().strftime("%Y-%m-%d")

            # Create the document to insert
            document = {
                "humidity": humidity,
                "pressure": pressure,
                "particulate_matter": particulate_matter,
                "temperature": temperature,
                "time": current_time,
                "date": current_date,
                "timestamp": time.time()
            }
            
            # Insert data into MongoDB
            collection.insert_one(document)
            print("Data sent to MongoDB:", document)
            
    except Exception as e:
        print(f"Error: {e}")
        # Handle error as needed (e.g., retry, log, etc.)

    time.sleep(1)  # Adjust sleep time as needed
