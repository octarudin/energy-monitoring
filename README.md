# Energy Monitoring
Single Phase Electrical Energy Monitoring with ESP32 or simply Energy Monitoring is a mini-project that intended to my portfolio. Also, to measure my skill while using ESP32.
In this project, FreeRTOS is used to get All Power Meter Data. Still available to add more device/sensor with RS-485 Communication. The data monitored in Local Server by MQTT Explorer. 
The status of monitoring is appended by timestamp so that the users can know when the controller starts to Online/Offline. 

To do this project, you need:
- ESP32 Microcontroller, 1 unit
- PZEM Power Meter, 1 unit
- AC Load, 1 unit
- TTL-RS485 Module, 1 unit
- Female-Female 4pin Dupont/Jumper Cable, 1 unit

# How to Wire
1. Connect ESP32 Uart1 to TTL-RS485 Module. Do not forget to cross the RX-TX.
2. Connect the A-B of TTL-RS485 module with A-B Power Meter in OFF Condition.
3. If you use Open/Split/External CT, then connect to the AC Cable (L or N) of AC load.
4. Connect ESP32 to your PC/Laptop to program.
5. Connect the Power Meter with your Electrical Equipments.
6. Turn ON the Power Meter.

Or simply follow this picture.  
<img src="images/Wiring PZEM-016.jpg" alt="How to Wire" width="500">

Enjoy.
