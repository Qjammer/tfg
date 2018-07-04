#!/usr/bin/python
import matplotlib.pyplot as plt
import csv
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
fig=plt.figure()
ax2=fig.add_subplot(111,projection='3d')
#ax1=fig.add_subplot(122)
ax2.set_xlabel('x')
ax2.set_ylabel('y')
ax2.set_zlabel('z')

nx=[]
ny=[]
w=[]
with open('data/buckets3.txt','r') as csvdata:
    plots=csv.reader(csvdata,delimiter=' ')
    for row in plots:
        #print (row)
        nx.append(int(row[0]))
        ny.append(int(row[1]))
        w.append(float(row[2]))

nxar=np.asarray(nx)
nxmax=np.amax(nxar)
nxmin=np.amin(nxar)
nyar=np.asarray(ny)
nymax=np.amax(nyar)
nymin=np.amin(nyar)
war=np.asarray(w)
wmax=np.amax(war)
wmin=np.amin(war)


x=[]
y=[]
z=[]
with open('data/points3.txt','r') as csvdata:
    plots=csv.reader(csvdata,delimiter=',')
    for row in plots:
        x.append(float(row[0]))
        y.append(float(row[1]))
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


argrid=np.ones((nymax-nymin+1,nxmax-nxmin+1))*2
for i in range(len(nxar)):
    argrid[nyar[i]-nymin,nxar[i]-nxmin]=war[i]

xp=np.linspace((nxmin-0.5)*0.5,(nxmax+0.5)*0.5,nxmax-nxmin+2)
yp=np.linspace((nymin-0.5)*0.5,(nymax+0.5)*0.5,nymax-nymin+2)
XX,YY=np.meshgrid(xp,yp)
ZZ=XX-XX+zmin



argrid=(argrid)
print(argrid)
ax2.set_xlim((nxmax+0.5)*0.5,(nxmin-0.5)*0.5)
ax2.set_ylim((nymax+0.5)*0.5,(nymin-0.5)*0.5)

surf=ax2.plot_surface(XX,YY,ZZ,vmin=1,vmax=4,rstride=1,cstride=1,cmap=plt.cm.bwr,facecolors=plt.cm.bwr((argrid-1)/4), shade=False)
m=plt.cm.ScalarMappable(cmap=plt.cm.bwr)
m.set_array(war)
m.set_clim(vmin=1,vmax=4)
PCM=ax2.get_children()[2]
fig.colorbar(m,shrink=0.5,aspect=5)

ax2.scatter(x,y,z,c='g',s=10,marker='.')

px=[]
py=[]
with open('data/path3.txt','r') as csvdata:
    plots=csv.reader(csvdata,delimiter=' ')
    for row in plots:
        #print (row)
        px.append(0.5*float(row[0]))
        py.append(0.5*float(row[1]))

ax2.plot(px,py,'m')









plt.show()
