import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv('pose_log.csv')

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))

ax1.plot(df['elapsed_s'], df['y'], label='y')
ax1.plot(df['elapsed_s'], df['z'], label='z')
ax1.set_xlabel('Time (s)')
ax1.set_ylabel('Position')
ax1.set_title('Y/Z over Time')
ax1.legend()
ax1.grid(True)

ax2.plot(df['elapsed_s'], df['angle'], color='orange')
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Angle (rad)')
ax2.set_title('Angle over Time')
ax2.grid(True)

plt.tight_layout()
plt.savefig('pose_plot.png', dpi=150)
plt.show()
