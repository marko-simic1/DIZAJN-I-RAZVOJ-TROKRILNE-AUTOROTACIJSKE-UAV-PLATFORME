import csv
import matplotlib.pyplot as plt

# Učitavanje samo Z vrijednosti iz CSV-a
z_vals = []

with open('imu_rec5_20.csv', newline='') as f:
    reader = csv.reader(f)
    for row in reader:
        try:
            z = float(row[3])
            z_vals.append(z)
        except ValueError:
            continue 


time_vals = [i * 0.01 for i in range(len(z_vals))]

plt.figure(figsize=(10, 4))
plt.plot(time_vals, z_vals, 'b-', label='Z (line)')
plt.title("Z vrijednost kroz vrijeme (line plot)")
plt.xlabel("Vrijeme (s)")
plt.ylabel("Z")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()

plt.figure(figsize=(10, 4))
plt.scatter(time_vals, z_vals, color='red', s=10, label='Z (scatter)')
plt.title("Z vrijednost kroz vrijeme (scatter plot)")
plt.xlabel("Vrijeme (s)")
plt.ylabel("Z")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()
