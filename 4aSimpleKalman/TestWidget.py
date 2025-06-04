import GenTestSig
import SimpleKalman

import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
import numpy as np


length = 500
value = 25
std = 3
rng = np.random.default_rng(seed=45)

# Initial parameters
A = 1.0
H = 1.0
Q = 0.0
R = 4.0
x_e = 6.0
P_0 = 2.0

def run_kalman(A, H, Q, R, x_e, P_0):
    test_signal = GenTestSig.get(value, rng, std=std, size=length)
    filtered_signal = SimpleKalman.filter(test_signal, x_i=x_e, p_i=P_0, A=A, H=H, Q=Q, R=R)
    return test_signal, filtered_signal

test_signal, filtered_signal = run_kalman(A, H, Q, R, x_e, P_0)

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 6))
plt.subplots_adjust(left=0.1, bottom=0.35)
l1 = ax1.scatter(np.arange(length), test_signal, label='Noisy Signal', color='blue', alpha=0.1)
l2, = ax1.plot(filtered_signal, label='Kalman Filtered Signal', color='orange')
ax1.hlines(value, 0, length, colors='green', linestyles='dashed', label='True Value')
ax1.set_xlabel('Sample Index')
ax1.set_ylabel('Value')
ax1.set_title('Kalman Filter Example')
ax1.legend()

l3, = ax2.plot(abs(filtered_signal - value), label='Filtered Signal - True Value', color='orange')
ax2.set_xlabel('Sample Index')
ax2.set_ylabel('Value Difference')
ax2.set_title('Difference from True Value')
ax2.axhline(0, color='red', linestyle='--', label='Zero Line')
ax2.legend()

# Slider axes
axcolor = 'lightgoldenrodyellow'
ax_A = plt.axes([0.1, 0.25, 0.8, 0.03], facecolor=axcolor) # x and y position, width, height
ax_H = plt.axes([0.1, 0.21, 0.8, 0.03], facecolor=axcolor)
ax_Q = plt.axes([0.1, 0.17, 0.8, 0.03], facecolor=axcolor)
ax_R = plt.axes([0.1, 0.13, 0.8, 0.03], facecolor=axcolor)
ax_xe = plt.axes([0.1, 0.09, 0.8, 0.03], facecolor=axcolor)
ax_P0 = plt.axes([0.1, 0.05, 0.8, 0.03], facecolor=axcolor)

s_A = Slider(ax_A, 'A', 0, 2, valinit=A) # Axes for slider, label, min, max, initial value
s_H = Slider(ax_H, 'H', 0, 2, valinit=H)
s_Q = Slider(ax_Q, 'Q', 0, 2, valinit=Q)
s_R = Slider(ax_R, 'R', 0, 10.0, valinit=R)
s_xe = Slider(ax_xe, 'x_e', 0.0, 50, valinit=x_e)
s_P0 = Slider(ax_P0, 'P_0', 0.0, 10.0, valinit=P_0)

def update(val):
    A = s_A.val
    H = s_H.val
    Q = s_Q.val
    R = s_R.val
    x_e = s_xe.val
    P_0 = s_P0.val
    _, filtered_signal = run_kalman(A, H, Q, R, x_e, P_0)
    l2.set_ydata(filtered_signal)
    l3.set_ydata(abs(filtered_signal - value))
    fig.canvas.draw_idle()

s_A.on_changed(update)
s_H.on_changed(update)
s_Q.on_changed(update)
s_R.on_changed(update)
s_xe.on_changed(update)
s_P0.on_changed(update)

plt.show()