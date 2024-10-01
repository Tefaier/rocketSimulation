from Simulation.Simulation import startSimulation
import pandas
import matplotlib.pyplot as plt

def func(n):
    t = []
    d = []
    k = 0
    for i in startSimulation(timeUnit=pandas.Timedelta(minutes=0, seconds=n)):
        if (k * n >= 3000):
            break
        t.append(i[0].total_seconds())
        for j in i[1]:
            if (k * n >= 3000):
                break
            if j[0] == 'Earth':
                earthX = j[1][0]
                earthY = j[1][1]
                earthZ = j[1][2]
            if j[0] == 'Rocket':
                d.append((j[1][0] - earthX) ** 2 + (j[1][1] - earthY) ** 2 + (j[1][2] - earthZ) ** 2)
                k += 1
    return [t, d]

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
plt.ylabel('Ось d, дистанция в км')
plt.title('График дистанции между ракетой и Землёй в зависимости от времени')
plt.show()
