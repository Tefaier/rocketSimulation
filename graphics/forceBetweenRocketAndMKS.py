from Simulation.Simulation import startSimulation
import pandas
import matplotlib.pyplot as plt

def func(n):
    t = []
    f = []
    k = 0
    for i in startSimulation(timeUnit=pandas.Timedelta(minutes=0, seconds=n)):
        if (k * n >= 300):
            break
        t.append(i[0].total_seconds())
        for j in i[1]:
            if (k * n >= 300):
                break
            if j[0] == 'Rocket':
                rocketX = j[1][0]
                rocketY = j[1][1]
                rocketZ = j[1][2]
            if j[0] == 'MKS':
                dist = (j[1][0] - rocketX) ** 2 + (j[1][1] - rocketY) ** 2 + (j[1][2] - rocketZ) ** 2
                g = 6.67 * 10 ** -11
                m1 = 773_000
                m2 = 440000
                f.append(g * m1 * m2 / dist)
                k += 1
    return [t, f]

# в комментариях написан промежуток времени между соседними кадрами
arr1 = func(60)
plt.plot(arr1[0], arr1[1], color='green') # 60 секунд
arr2 = func(30)
plt.plot(arr2[0], arr2[1], color='blue') # 30 секунд
arr3 = func(20)
plt.plot(arr3[0], arr3[1], color='yellow') # 20 секунд
arr4 = func(12)
plt.plot(arr4[0], arr4[1], color='red') # 12 секунд
arr5 = func(6)
plt.plot(arr5[0], arr5[1], color='orange') # 6 секунд

plt.xlabel('Ось t, время в секундах')
plt.ylabel('Ось f, сила в Ньютонах')
plt.title('График силы притяжения ракеты и МКС в зависимости от времени')
plt.show()
