#!/usr/bin/python
import math
import matplotlib.pyplot as plt

a = 0
w = 0
u=0
v=0
amp = 1
uu = dict()
vv = dict()
ww = dict()

UU = dict()
VV = dict()
WW = dict()
while a < 360:
    U = amp * math.sin(math.radians(a))
    V = amp * math.sin(math.radians(a+120))
    W = amp * math.sin(math.radians(a+240))
    #print 'a:' + str(a) +' U: ' + str(U) + ' V: ' + str(V) + ' W: ' + str(W)

    # u v w her zaman >0 ve <1 olmali
    # U = v - u
    # V = w - v
    # W = u - w
    if a <= 60:
        #U+ V+ W-
        #u sabit
        u = 0
        v = u + U
        w = u - W
    elif a <= 120:
        #U+ V- W-
        #v sabit
        v = 1
        u = v - U
        w = V + v
    elif a <= 180:
        #U+ V- W+
        #w sabit
        w = 0
        u = W + w
        v = w - V 
    elif a <= 240:
        #U- V- W+
        #u sabit
        u = 1
        v = u + U
        w = u - W
    elif a <= 300:
        #U- V+ W+
        #v sabit
        v = 0
        u = v - U
        w = V + v
    elif a <= 360:
        #U- V+ W-
        #w sabit
        w = 1
        u = W + w
        v = w - V 

    uu[a] = u
    vv[a] = v
    ww[a] = w
    UU[a] = v-u
    VV[a] = w-v
    WW[a] = u-w
    a = a + 1
    if v < 0 or v > 1 or u < 0 or u > 1 or w < 0 or w > 1:
        print 'hata'
    print 'a: '+str(a)+' u: ' + str(u) + ' v: ' + str(v) + ' w: ' + str(w)
    print 'Vu: ' + str(v-u) + ' Vv: ' + str(w-v) + ' Vw: ' + str(u-w)
plt.plot(range(0,360),uu.values(),range(0,360),vv.values(),range(0,360),ww.values());
plt.show()
plt.plot(range(0,360),UU.values(),range(0,360),VV.values(),range(0,360),WW.values());
plt.show()

