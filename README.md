# ESP-Farm-System
Auto Farming System ESP-32
Summary: 
This system is designed to automate watering, lighting, and sensor data collection 
for household or small community greenhouse farming. It can be easily monitored 
and controlled through any internet-connected device. Powered by four ESP-32 
devices, the system collects environmental data, processes it using moving 
averages and low-pass filters for accuracy, and sends this data to other devices to 
activate watering and lighting systems as needed. Users can customize the 
frequency of data collection, ranging from once a day to every 15 minutes, 
optimizing power usage based on plant care needs. The data is sent to Adafruit IO 
which graphs all data, this can be analyzed and be used to improve the system 
itself.  
  
The system includes four key ESP-32 units. SS.ESP handles sensor data collection 
and sends it to the rest of the system. H.ESP activates the watering system based 
on sensor readings, ensuring plants receive the correct amount of water. L.ESP 
manages the lighting system, providing the lights turn on at a fixed time each day. 
Sunflower tracks the sun's position by adjusting its motor to align with the azimuth 
and elevation of the sun, using monthly average data from a sun-tracking website. 
This feature makes the system self-sufficient. The 3D model design of
