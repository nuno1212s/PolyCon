import matplotlib.pyplot as plt
import json

import matplotlib.colors as mcolors
import sys

scale = 1/20

if len(sys.argv) != 3:
    print("Insert M:")

    M = int(input())

    print("Insert the result.json file location:")
    file = input()
else:
    M = int(sys.argv[1])
    file = sys.argv[2]

f = open(file)

lines = f.read()

result = json.loads(lines)

print(result)

fig, ax = plt.subplots(figsize=(M * scale, M * scale))

ax.set(xlim=(0, M ), ylim=(0, M ))

print(mcolors.BASE_COLORS)

colorList = list(mcolors.BASE_COLORS.keys())[:-1]

print(colorList)

current = 0
for triangle in result[:-1]:
    tPointsX, tPointsY = [], []

    for tPoint in triangle:
        tPointsX.append(tPoint[0])
        tPointsY.append(tPoint[1])

    tPointsX.append(triangle[0][0])
    tPointsY.append(triangle[0][1])

    print("Plotting " + str(tPointsX) + " " + str(tPointsY) + " with color " + colorList[current % len(colorList)])
    ax.fill(tPointsX, tPointsY, color=mcolors.BASE_COLORS[colorList[current % len(colorList)]])

    current+=1

plt.draw()
plt.show()
