import matplotlib.pyplot as plt
import csv
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
#fig=plt.figure()
#ax=fig.add_subplot(111,projection='2d')
#ax.set_xlabel('x')
#ax.set_ylabel('y')

x=[]
y=[]
z=[]
with open('../build/buckets3.txt','r') as csvdata:
    plots=csv.reader(csvdata,delimiter=' ')
    for row in plots:
        #print (row)
        x.append(int(row[0]))
        y.append(int(row[1]))
        z.append(float(row[2]))


xar=np.asarray(x)
xmax=np.amax(xar)
xmin=np.amin(xar)
yar=np.asarray(y)
ymax=np.amax(yar)
ymin=np.amin(yar)
zar=np.asarray(z)
zmax=np.amax(zar)
zmin=np.amin(zar)

print(xmax)
print(xmin)
print(ymax)
print(ymin)
argrid=np.ones((ymax-ymin+1,xmax-xmin+1))*2

for i in range(len(xar)):
    argrid[yar[i]-ymin,xar[i]-xmin]=zar[i]


argrid=(argrid)
print(argrid)
#plt.ylim(ymax,ymin)
plt.imshow(argrid,vmin=1,vmax=4,extent=[xmin-0.5,xmax+0.5,ymin-0.5,ymax+0.5],cmap=plt.cm.bwr,origin='lower')
plt.colorbar()
plt.show()
