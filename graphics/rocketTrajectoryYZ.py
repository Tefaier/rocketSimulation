from Simulation.Simulation import startSimulation
import pandas
import matplotlib.pyplot as plt

def func(n, bool=True):
    x = []
    y = []
    z = []
    k = 0
    for i in startSimulation(timeUnit=pandas.Timedelta(minutes=0, seconds=n), applyAtmosphere=bool):
        for j in i[1]:
            if (k * n >= 3000):
                break
            if j[0] == 'Earth':
                earthX = j[1][0]
                earthY = j[1][1]
                earthZ = j[1][2]
            if j[0] == 'Rocket':
                x.append(j[1][0] - earthX)
                y.append(j[1][1] - earthY)
                z.append(j[1][2] - earthZ)
                k += 1
    return [x, y, z]

# в комментариях написан промежуток времени между соседними кадрами

# с атмосферой
arr11 = func(50)
plt.plot(arr11[1], arr11[2], color='green') # 50 секунд
arr21 = func(30)
plt.plot(arr21[1], arr21[2], color='blue') # 30 секунд
arr31 = func(20)
plt.plot(arr31[1], arr31[2], color='yellow') # 20 секунд
arr41 = func(12)
plt.plot(arr41[1], arr41[2], color='red') # 12 секунд
arr51 = func(6)
plt.plot(arr51[1], arr51[2], color='orange') # 6 секунд

# без атмосферы (пунктирная линия)
arr12 = func(50, False)
plt.plot(arr12[1], arr12[2], '--', color='green') # 50 секунд
arr22 = func(30, False)
plt.plot(arr22[1], arr22[2], '--', color='blue') # 30 секунд
arr32 = func(20, False)
plt.plot(arr32[1], arr32[2], '--', color='yellow') # 20 секунд
arr42 = func(12, False)
plt.plot(arr42[1], arr42[2], '--', color='red') # 12 секунд
arr52 = func(6, False)
plt.plot(arr52[1], arr52[2], '--', color='orange') # 6 секунд

plt.xlabel('Ось y')
plt.ylabel('Ось z')
plt.title('График координат ракеты относительно центра Земли в плоскости YZ')
plt.show()
