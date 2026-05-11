from NetFT import Sensor         # pip install NetFT
import time
import matplotlib.pyplot as plt
import serial
from scipy.io import savemat

######################## load cell setup ###########################
ip = "192.168.1.1"
sensor45 = Sensor(ip)
sensor45.tare(100)   # average 100 samples
print("Tare complete")

forcePerCount = 1000000 # for mini45
torquePerCount = 1000000 # for mini45

######################## test/display settings #####################
printSetting = 1  # 1 = print in terminal, 0 = don't print 
duration = 50  # seconds
sample_rate = 50.0  # Hz

######################## data collection function ###########################
def collect_data():
    t_vals = []
    fx_vals, fy_vals, fz_vals = [], [], []
    tx_vals, ty_vals, tz_vals = [], [], []
    
    dt = 1.0 / sample_rate

    start_time = time.perf_counter()
    next_sample_time = start_time

    try:
        while (time.perf_counter() - start_time) < duration:
            now = time.perf_counter()

            if now >= next_sample_time:
                current_time = now - start_time

                data = sensor45.getMeasurement()
                fx, fy, fz, tx, ty, tz = data

                # Convert units
                fx /= forcePerCount
                fy /= forcePerCount
                fz /= forcePerCount
                tx /= torquePerCount
                ty /= torquePerCount
                tz /= torquePerCount

                # Store data
                t_vals.append(current_time)
                fx_vals.append(fx)
                fy_vals.append(fy)
                fz_vals.append(fz)
                tx_vals.append(tx)
                ty_vals.append(ty)
                tz_vals.append(tz)

                if printSetting == 1:
                    print(f"t:{current_time:.4f} | Fx:{fx:.2f}, Fy:{fy:.2f}, Fz:{fz:.2f}, Tx:{tx:.2f}, Ty:{ty:.2f}, Tz:{tz:.2f}")

                next_sample_time += dt

            else:
                # sleep just a tiny bit to avoid CPU burn
                time.sleep(0.001)

    except KeyboardInterrupt:
        print("Stopping early")

    return t_vals, fx_vals, fy_vals, fz_vals, tx_vals, ty_vals, tz_vals

######################## Trigger setup ###########################
ser = serial.Serial('COM3', 115200, timeout=20)
print("Waiting for trigger from Elegoo...")
while True:
    line = ser.readline().decode('utf-8').strip()
    if line == "START":
        print(f"Triggered! Starting data collection at {time.time()}")
        t_vals, fx_vals, fy_vals, fz_vals, tx_vals, ty_vals, tz_vals = collect_data()
        break
ser.close()

###################### save data as .mat ####################
savemat("loadcell_data.mat", {
    "t": t_vals,
    "Fx": fx_vals,
    "Fy": fy_vals,
    "Fz": fz_vals,
    "Tx": tx_vals,
    "Ty": ty_vals,
    "Tz": tz_vals,
    "sample_rate": sample_rate
})

######################## plotting ###########################
plt.figure(figsize=(10, 8))

# Forces
plt.subplot(2, 1, 1)
plt.plot(t_vals, fx_vals, label='Fx')
plt.plot(t_vals, fy_vals, label='Fy')
plt.plot(t_vals, fz_vals, label='Fz')
plt.ylabel("Force (N)")
plt.legend()
plt.grid()

# Torques
plt.subplot(2, 1, 2)
plt.plot(t_vals, tx_vals, label='Tx')
plt.plot(t_vals, ty_vals, label='Ty')
plt.plot(t_vals, tz_vals, label='Tz')
plt.xlabel("Time (s)")
plt.ylabel("Torque (Nm)")
plt.legend()
plt.grid()

plt.tight_layout()
plt.show()
