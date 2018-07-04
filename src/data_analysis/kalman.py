#!/usr/bin/python
import matplotlib.pyplot as plt
import csv
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
#fig=plt.figure()
#ax=fig.add_subplot(111,projection='2d')
#ax.set_xlabel('x')
#ax.set_ylabel('y')

t=[]
rotvel=[]
gyr=[]
tac=[]

with open('data/state_data.txt','r') as csvdata:
    plots=csv.reader(csvdata,delimiter='\t')
    for row in plots:
        if len(row)>0:
            if row[0]=="t":
                if float(row[1])>7.4:
                    break
                t.append(float(row[1]))
            elif row[0]=="rotvel":
                rotvel.append(float(row[3]))
            elif row[0]=="tac":
                tac.append([float(row[1]),float(row[2]),float(row[3]),float(row[4])])
            elif row[0]=="gyr":
                gyr.append(float(row[3]));


tacestimate=[];
for tacpt in tac:
    vl=(tacpt[0]+tacpt[1])/2;
    vr=(tacpt[2]+tacpt[3])/2;
    r1r2=0.4;
    tacestimate.append((vr-vl)/r1r2);

plt.plot(t,[-0.45*x for x in gyr],linewidth=0.5);
plt.plot(t,[-0.16*x for x in tacestimate],linewidth=0.5);
plt.plot(t,[-1*x for x in rotvel]);
plt.show();
