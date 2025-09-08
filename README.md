# Inline Temperature Regulator

This project enables **temperature control** for an electric heater that originally had no built-in regulation.  
It uses an **Arduino Nano**, an **AHT10 temperature/humidity sensor**, a **rotary encoder with button**, an **SSD1306 OLED display**, and a **relay** to switch the heater.

---

## Background  

When my best friend moved into his new apartment, the bathroom heater had **no temperature control**:  
- If left **on all the time** → very high energy bills.  
- If left **off too often** → risk of mold due to humidity.  

I built this regulator as a **quick, practical, and effective solution** mostly from spare parts.  

---

## Photos  

<img width="1307" height="1009" alt="overview" src="https://github.com/user-attachments/assets/c8161f2c-85ee-4c19-9ed9-f300a58b5139" />

![20250908_082758293_iOS](https://github.com/user-attachments/assets/9f64083a-4a5d-4183-b8d4-e79ee7715055)  
![20250908_082736350_iOS](https://github.com/user-attachments/assets/102e3405-8c76-4bb9-ba88-c8bbfe1b264b)  
![20250908_082718428_iOS](https://github.com/user-attachments/assets/a05c7e54-4754-4a00-b8ea-de2914e5e7e0)  

---

## Features  

- ✅ Adjustable **temperature setpoint** with rotary encoder  
- ✅ **Sleep mode** + wake on encoder/button or watchdog timer  
- ✅ **OLED display** shows target temp, current temp, humidity, relay status, and daily energy use  
- ✅ **Relay control** with hysteresis (prevents fast switching)  
- ✅ **Energy estimation** (approximate Wh/day from current sensor)  
- ✅ Auto **display-off after ~30 s** of inactivity  

---

## Hardware Overview  

| Function           | Arduino Pin | Notes |
|--------------------|-------------|-------|
| Relay output       | D5 (PD5)    | Controls heater via relay/SSR |
| Current sensor     | A7 (ADC7)   | For approximate energy measurement |
| Encoder switch     | D2 (INT0)   | Wakes device + user input |
| Encoder DT         | D9 (PB1)    | Used for direction |
| Encoder CLK        | D10 (PB2)   | Pin change interrupt |
| OLED + AHT10       | A4/A5 (I²C) | 0x3C (SSD1306), 0x38 (AHT10) |

⚠️ **Important safety note**:  
This project switches **mains AC**. Use an **isolated relay or SSR**, include a **flyback diode** if using a coil relay, and put everything in a safe enclosure. For bathroom use, ensure IP-rated housing and an independent **thermal fuse** on the heater.  

---

## Configuration  

Some important constants in the code:  

```cpp
int targetTemperature = 250; // Default = 25.0 °C (×10 scaling)
const int hysteresis = 5;    // ±0.5 °C hysteresis
const float voltage_Vrms = 230.0; // Adjust for your mains (120/230 V)

// Display auto-off duration (~8 s per watchdog tick)
#define DISPLAY_ON_DURATION_WDT_COUNTS 4

// Current sensor calibration (replace with measured values)
const float calibration_factor = 0.1;
const float calibration_offset = 0.0;
```

- **Setpoint range**: 10.0–30.0 °C (`constrain(..., 100, 300)`)  
- **Display timeout**: `4 × 8 s ≈ 32 s`  
- **Energy reset**: Every 24 h (based on internal counter, no RTC)  

---

## How It Works  

1. **Wake-up**: Encoder/button wakes device from sleep.  
2. **Set temperature**: Rotate encoder to change target.  
3. **Relay control**:  
   - Heater ON if below `(setpoint - hysteresis)`  
   - Heater OFF if above `(setpoint + hysteresis)`  
4. **Display**: Shows set temp, current temp, humidity, relay state, and estimated daily energy.  
5. **Energy calculation**: Uses simple `P = Vrms × I` and integrates every 8 s. (Approximate, assumes resistive load).  

---

## Required Libraries  

- [Adafruit GFX](https://github.com/adafruit/Adafruit-GFX-Library)  
- [Adafruit SSD1306](https://github.com/adafruit/Adafruit_SSD1306)  
- [Adafruit AHTX0](https://github.com/adafruit/Adafruit_AHTX0)  
- Wire (built-in with Arduino)  

---

## Limitations  

- No **EEPROM storage** → setpoint resets after power cycle  
- No **RTC** → daily energy counter is relative, not synced to real time  
- Energy estimation is **approximate only** (no true RMS sampling)  
- Encoder may need **debouncing** if noisy  

---

## Future Improvements  

- Store settings in **EEPROM**  
- Add **RTC module** for proper timekeeping  
- Implement **true RMS current measurement**  
- Add dew-point calculation & mold-risk indicator  
- Add menu system for settings (hysteresis, timeout, etc.)  

---
