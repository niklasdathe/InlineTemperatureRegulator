# Inline Temperature Regulator

This project enables temperature control for an electric heater that originally had no built-in regulation.

---

## Background  

When my best friend moved into his new apartment, he discovered that the bathroom heater had **no temperature control**.  
- If the heater stayed plugged in, it could lead to **high energy bills**.  
- If it stayed unplugged, **mold could form** due to humidity.  

I decided to help out by building a **simple, practical, and effective solution** with parts I already had on hand.  

---

## Photos  

<img width="1307" height="1009" alt="overview" src="https://github.com/user-attachments/assets/c8161f2c-85ee-4c19-9ed9-f300a58b5139" />

![20250908_082758293_iOS](https://github.com/user-attachments/assets/9f64083a-4a5d-4183-b8d4-e79ee7715055)  
![20250908_082736350_iOS](https://github.com/user-attachments/assets/102e3405-8c76-4bb9-ba88-c8bbfe1b264b)  
![20250908_082718428_iOS](https://github.com/user-attachments/assets/a05c7e54-4754-4a00-b8ea-de2914e5e7e0)  

---

## Technical Challenges  

The project runs on a **simple Arduino Nano**, so I had to squeeze out as much performance as possible.  

- Implemented **sleep mode** to save power when idle.  
- The **rotary encoder input** had to feel responsive despite low power usage.  
- Added **hysteresis** to avoid frequent on/off switching of the heater.  

---

## How It Works  

1. **Wake-up**: Interact with the encoder to wake up the device.  
2. **Set Temperature**: Turn the encoder knob to select the desired temperature.  
3. **Control Logic**:  
   - If the actual temperature is **below the setpoint**, the heater turns **on**.  
   - If above, the heater turns **off**.  
   - A small hysteresis prevents rapid switching.  
4. **Humidity Monitoring**: Since the sensor also provides humidity data, it is displayed as well—useful for tracking mold risk.  

---

## Features  

- ✅ Adjustable temperature control  
- ✅ Automatic sleep/wake functionality  
- ✅ Energy-efficient operation  
- ✅ Temperature + humidity display  

---
