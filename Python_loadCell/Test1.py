from NetFT import Sensor         # pip install NetFT
import time
import matplotlib.pyplot as plt

######################## load cell setup ###########################
ip = "192.168.1.1"
sensor45 = Sensor(ip)

sensor45.tare(100)   # average 100 samples
print("Tare complete")

#forcePerCount = 224808.9 # for mini58, use 224808.9 for output in N
#torquePerCount = 8850746 # for mini58, use 8850746 for Nm

# mini45 FT46847 = SI-290-10
forcePerCount = 1000000 # for mini45, use 224808.9 for output in N
torquePerCount = 1000000 # for mini45, use 8850746 for Nm

######################## test/display settings #####################
printSetting = 1  # 1 = print in terminal, 0 = don't print 
duration = 30  # seconds

######################## data collection ###########################
t_vals = []
fx_vals, fy_vals, fz_vals = [], [], []
tx_vals, ty_vals, tz_vals = [], [], []

start_time = time.time()
try:
    while (time.time() - start_time) < duration:
        current_time = time.time() - start_time
        
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
            print(f"t:{current_time:.2f} | Fx:{fx:.2f}, Fy:{fy:.2f}, Fz:{fz:.2f}, Tx:{tx:.2f}, Ty:{ty:.2f}, Tz:{tz:.2f}")

        time.sleep(0.002)

except KeyboardInterrupt:
    print("stopping early")

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