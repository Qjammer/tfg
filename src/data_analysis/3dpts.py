import matplotlib.pyplot as plt
import csv
from mpl_toolkits.mplot3d import Axes3D
fig=plt.figure()
ax=fig.add_subplot(111,projection='3d')
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')

x=[]
y=[]
z=[]
with open('../build/points3.txt','r') as csvdata:
    plots=csv.reader(csvdata,delimiter=',')
    for row in plots:
        #print (row)
        x.append(float(row[0]))
        y.append(float(row[1]))
        z.append(float(row[2]))
ax.scatter(x,y,z,s=10,marker='.')
plt.gca().set_aspect('equal', adjustable='box')
plt.show()
