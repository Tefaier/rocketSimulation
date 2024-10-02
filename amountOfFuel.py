from Simulation.Simulation import startSimulation
import pandas
import matplotlib.pyplot as plt

def func(n, bool=True):
    t = []
    f = []
    k = 0
    for i in startSimulation(timeUnit=pandas.Timedelta(minutes=0, seconds=n), applyAtmosphere=bool):
        k += 1
        if (k * n >= 300):
            break
        t.append(i[0].total_seconds())
        f.append(i[2])
    return [t, f]

# в комментариях написан промежуток времени между соседними кадрами

# с атмосферой
arr11 = func(50)
plt.plot(arr11[0], arr11[1], color='green') # 50 секунд
arr21 = func(30)
plt.plot(arr21[0], arr21[1], color='blue') # 30 секунд
arr31 = func(20)
plt.plot(arr31[0], arr31[1], color='yellow') # 20 секунд
arr41 = func(12)
plt.plot(arr41[0], arr41[1], color='red') # 12 секунд
arr51 = func(6)
plt.plot(arr51[0], arr51[1], color='orange') # 6 секунд

# без атмосферы (пунктирная линия)
arr12 = func(50, False)
plt.plot(arr12[0], arr12[1], '--', color='green') # 50 секунд
arr22 = func(30, False)
plt.plot(arr22[0], arr22[1], '--', color='blue') # 30 секунд
arr32 = func(20, False)
plt.plot(arr32[0], arr32[1], '--', color='yellow') # 20 секунд
arr42 = func(12, False)
plt.plot(arr42[0], arr42[1], '--', color='red') # 12 секунд
arr52 = func(6, False)
plt.plot(arr52[0], arr52[1], '--', color='orange') # 6 секунд

plt.xlabel('Ось t, время в секундах')
plt.ylabel('Ось f, количество топлива в кг')
plt.title('График количества топлива в зависимости от времени')
plt.show()
