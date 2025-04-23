import numpy as np
import matplotlib.pyplot as plt

# x 값 범위 설정 (0보다 큰 값으로 설정)
x = np.linspace(0.1, 5, 5)
y = np.log(x)
z = x - 2.5

# 그래프 그리기
plt.figure(figsize=(8, 6))
plt.plot(x, y, z,label=r'$y = \ln(x) + 1$')
plt.xlabel('x')
plt.ylabel('y')
plt.title(r'Graph of $y = \ln(x) + 1$')
plt.axhline(0, color='black',linewidth=0.5)
plt.axvline(0, color='black',linewidth=0.5)
plt.grid(color = 'gray', linestyle = '--', linewidth = 0.5)
plt.legend()
plt.show()
