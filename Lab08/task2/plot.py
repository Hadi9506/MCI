import serial
import matplotlib.pyplot as plt
from drawnow import *

# === Setup Serial ===
sinWaveData = serial.Serial('/dev/ttyUSB0', 115200)
plt.ion()  # Enable interactive mode

adcValues = []
time_ms = []
cnt = 0  # Time index (1 ms steps)

# === Plotting Function ===
def makeFig():
    plt.clf()
    plt.title('Live ADC & Filtered Data')
    plt.grid(True)
    plt.xlabel('Time (ms)')
    plt.ylabel('ADC Value')
    plt.ylim(0, 40)  # Adjust range as needed
    plt.plot(time_ms, adcValues, 'r.-', label='Raw ADC')
    plt.legend(loc='upper left')

# === Main Loop ===
while True:
    while sinWaveData.inWaiting() == 0:
        pass  # Wait for data

    try: 
        line = sinWaveData.readline().decode().strip()
        values = line

        if len(values) == 2:
            raw = int(values[0])
            adcValues.append(raw)            
            time_ms.append(cnt)
            cnt += 1
            drawnow(makeFig)
            plt.pause(0.0001)
            if len(adcValues) > 500:
                adcValues.pop(0)
                time_ms.pop(0)
    except Exception as e:
        print("Error:", e)
