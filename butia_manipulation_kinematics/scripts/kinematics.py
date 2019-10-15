import numpy as np
import math

np.set_printoptions(suppress=True)

def calc_jacobian(angles):

    t0 = angles[0]
    t1 = angles[1]
    t2 = angles[2]
    t3 = angles[3]
    t4 = angles[4]


    a00 = -0.25*math.sin(t0)

    a01 = -0.215*(math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)

    a02 = -0.19*((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)*math.sin(t2)*math.cos(t3) + 0.014*((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)*math.cos(t2)

    a03 = -0.19*((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)*math.sin(t3)*math.cos(t2)

    a04 = 0.0
   

    a10 = 0.0

    a11 = 0.0

    a12 = 0.0

    a13 = 0.19*math.cos(t3)

    a14 = 0.0

    
    a20 = -0.25*math.cos(t0)

    a21 = -0.215*(math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)

    a22 = -0.014*((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)*math.sin(t2) - 0.19*((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)*math.cos(t2)*math.cos(t3)

    a23 = 0.19*((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)*math.sin(t2)*math.sin(t3)

    a24 = 0.0


    a30 = 0.0

    a31 = 0.0

    a32 = (-((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)*math.sin(t2)*math.sin(t4) + ((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)*math.sin(t3)*math.cos(t2)*math.cos(t4))*(-((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)*math.sin(t2)*math.sin(t3)*math.sin(t4) + ((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)*math.cos(t2)*math.cos(t4))/((-((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)*math.sin(t2)*math.sin(t3)*math.sin(t4) + ((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)*math.cos(t2)*math.cos(t4))**2 + (((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)*math.sin(t2)*math.sin(t3)*math.cos(t4) + ((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)*math.sin(t4)*math.cos(t2))**2) + (-((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)*math.sin(t2)*math.cos(t4) - ((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)*math.sin(t3)*math.sin(t4)*math.cos(t2))*(-((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)*math.sin(t2)*math.sin(t3)*math.cos(t4) - ((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)*math.sin(t4)*math.cos(t2))/((-((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)*math.sin(t2)*math.sin(t3)*math.sin(t4) + ((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)*math.cos(t2)*math.cos(t4))**2 + (((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)*math.sin(t2)*math.sin(t3)*math.cos(t4) + ((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)*math.sin(t4)*math.cos(t2))**2)

    a33 = ((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)*(-((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)*math.sin(t2)*math.sin(t3)*math.sin(t4) + ((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)*math.cos(t2)*math.cos(t4))*math.sin(t2)*math.cos(t3)*math.cos(t4)/((-((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)*math.sin(t2)*math.sin(t3)*math.sin(t4) + ((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)*math.cos(t2)*math.cos(t4))**2 + (((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)*math.sin(t2)*math.sin(t3)*math.cos(t4) + ((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)*math.sin(t4)*math.cos(t2))**2) - ((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)*(-((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)*math.sin(t2)*math.sin(t3)*math.cos(t4) - ((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)*math.sin(t4)*math.cos(t2))*math.sin(t2)*math.sin(t4)*math.cos(t3)/((-((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)*math.sin(t2)*math.sin(t3)*math.sin(t4) + ((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)*math.cos(t2)*math.cos(t4))**2 + (((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)*math.sin(t2)*math.sin(t3)*math.cos(t4) + ((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)*math.sin(t4)*math.cos(t2))**2)

    a34 = (-((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)*math.sin(t2)*math.sin(t3)*math.sin(t4) + ((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)*math.cos(t2)*math.cos(t4))**2/((-((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)*math.sin(t2)*math.sin(t3)*math.sin(t4) + ((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)*math.cos(t2)*math.cos(t4))**2 + (((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)*math.sin(t2)*math.sin(t3)*math.cos(t4) + ((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)*math.sin(t4)*math.cos(t2))**2) + (-((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)*math.sin(t2)*math.sin(t3)*math.cos(t4) - ((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)*math.sin(t4)*math.cos(t2))**2/((-((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)*math.sin(t2)*math.sin(t3)*math.sin(t4) + ((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)*math.cos(t2)*math.cos(t4))**2 + (((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)*math.sin(t2)*math.sin(t3)*math.cos(t4) + ((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)*math.sin(t4)*math.cos(t2))**2)

 
    a40 = 0.0

    a41 = 0.0

    a42 = ((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)*math.sin(t2)*math.sin(t3)*math.cos(t3)/(((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)**2*math.cos(t2)**2*math.cos(t3)**2 + math.sin(t3)**2)

    a43 = ((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)*math.sin(t3)**2*math.cos(t2)/(((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)**2*math.cos(t2)**2*math.cos(t3)**2 + math.sin(t3)**2) + ((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)*math.cos(t2)*math.cos(t3)**2/(((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)**2*math.cos(t2)**2*math.cos(t3)**2 + math.sin(t3)**2)

    a44 = 0.0


    a50 = 0.0

    a51 = 0.0

    a52 = ((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)*(((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)**2*math.cos(t2)**2*math.cos(t3)**2/math.sqrt(((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)**2*math.cos(t2)**2*math.cos(t3)**2 + math.sin(t3)**2) + math.sin(t3)**2/math.sqrt(((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)**2*math.cos(t2)**2*math.cos(t3)**2 + math.sin(t3)**2))*math.cos(t2)*math.cos(t3)/(((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)**2*math.sin(t2)**2*math.cos(t3)**2 + (((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)**2*math.cos(t2)**2*math.cos(t3)**2/math.sqrt(((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)**2*math.cos(t2)**2*math.cos(t3)**2 + math.sin(t3)**2) + math.sin(t3)**2/math.sqrt(((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)**2*math.cos(t2)**2*math.cos(t3)**2 + math.sin(t3)**2))**2) - ((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)*(((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)**4*math.sin(t2)*math.cos(t2)**3*math.cos(t3)**4/(((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)**2*math.cos(t2)**2*math.cos(t3)**2 + math.sin(t3)**2)**(3/2) - 2*((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)**2*math.sin(t2)*math.cos(t2)*math.cos(t3)**2/math.sqrt(((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)**2*math.cos(t2)**2*math.cos(t3)**2 + math.sin(t3)**2) + ((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)**2*math.sin(t2)*math.sin(t3)**2*math.cos(t2)*math.cos(t3)**2/(((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)**2*math.cos(t2)**2*math.cos(t3)**2 + math.sin(t3)**2)**(3/2))*math.sin(t2)*math.cos(t3)/(((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)**2*math.sin(t2)**2*math.cos(t3)**2 + (((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)**2*math.cos(t2)**2*math.cos(t3)**2/math.sqrt(((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)**2*math.cos(t2)**2*math.cos(t3)**2 + math.sin(t3)**2) + math.sin(t3)**2/math.sqrt(((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)**2*math.cos(t2)**2*math.cos(t3)**2 + math.sin(t3)**2))**2)

    a53 = -((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)*(((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)**2*math.cos(t2)**2*math.cos(t3)**2/math.sqrt(((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)**2*math.cos(t2)**2*math.cos(t3)**2 + math.sin(t3)**2) + math.sin(t3)**2/math.sqrt(((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)**2*math.cos(t2)**2*math.cos(t3)**2 + math.sin(t3)**2))*math.sin(t2)*math.sin(t3)/(((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)**2*math.sin(t2)**2*math.cos(t3)**2 + (((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)**2*math.cos(t2)**2*math.cos(t3)**2/math.sqrt(((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)**2*math.cos(t2)**2*math.cos(t3)**2 + math.sin(t3)**2) + math.sin(t3)**2/math.sqrt(((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)**2*math.cos(t2)**2*math.cos(t3)**2 + math.sin(t3)**2))**2) - ((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)*(-2*((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)**2*math.sin(t3)*math.cos(t2)**2*math.cos(t3)/math.sqrt(((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)**2*math.cos(t2)**2*math.cos(t3)**2 + math.sin(t3)**2) + ((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)**2*(((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)**2*math.sin(t3)*math.cos(t2)**2*math.cos(t3) - math.sin(t3)*math.cos(t3))*math.cos(t2)**2*math.cos(t3)**2/(((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)**2*math.cos(t2)**2*math.cos(t3)**2 + math.sin(t3)**2)**(3/2) + 2*math.sin(t3)*math.cos(t3)/math.sqrt(((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)**2*math.cos(t2)**2*math.cos(t3)**2 + math.sin(t3)**2) + (((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)**2*math.sin(t3)*math.cos(t2)**2*math.cos(t3) - math.sin(t3)*math.cos(t3))*math.sin(t3)**2/(((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)**2*math.cos(t2)**2*math.cos(t3)**2 + math.sin(t3)**2)**(3/2))*math.sin(t2)*math.cos(t3)/(((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)**2*math.sin(t2)**2*math.cos(t3)**2 + (((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)**2*math.cos(t2)**2*math.cos(t3)**2/math.sqrt(((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)**2*math.cos(t2)**2*math.cos(t3)**2 + math.sin(t3)**2) + math.sin(t3)**2/math.sqrt(((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)**2*math.cos(t2)**2*math.cos(t3)**2 + math.sin(t3)**2))**2)

    a54 = 0.0
    

    J = np.array([   [a00, a01, a02, a03, a04], \
                    [a10, a11, a12, a13, a14], \
                    [a20, a21, a22, a23, a24], \
                    [a30, a31, a32, a33, a34], \
                    [a40, a41, a42, a43, a44], \
                    [a50, a51, a52, a53, a54] 
                ])

    return(J)

def forwardKinematics(angles):

    t0 = angles[0]
    t1 = angles[1]
    t2 = angles[2]
    t3 = angles[3]
    t4 = angles[4]

    x = 0.014*((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)*math.sin(t2) + 0.19*((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)*math.cos(t2)*math.cos(t3) + 0.0225*(math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + 0.0225*(math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2 + 0.215*(math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1) + 0.1045*math.sin(t0)**2 + 0.1045*math.cos(t0)**2 + 0.25*math.cos(t0) - 0.01105


    y = 0.19*math.sin(t3)


    z = -0.19*((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)*math.sin(t2)*math.cos(t3) + 0.014*((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)*math.cos(t2) - 0.024*(math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 - 0.215*(math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1) - 0.024*(math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2 - 0.25*math.sin(t0) + 0.9635


    alpha = math.atan2(((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)*math.sin(t2)*math.sin(t3)*math.cos(t4) + ((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)*math.sin(t4)*math.cos(t2), -((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)*math.sin(t2)*math.sin(t3)*math.sin(t4) + ((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)*math.cos(t2)*math.cos(t4))


    beta = math.atan2(math.sin(t3), ((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)*math.cos(t2)*math.cos(t3))


    gamma = math.atan2(((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)*math.sin(t2)*math.cos(t3), ((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)**2*math.cos(t2)**2*math.cos(t3)**2/math.sqrt(((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)**2*math.cos(t2)**2*math.cos(t3)**2 + math.sin(t3)**2) + math.sin(t3)**2/math.sqrt(((math.sin(t0)**2 + math.cos(t0)**2)*math.sin(t1)**2 + (math.sin(t0)**2 + math.cos(t0)**2)*math.cos(t1)**2)**2*math.cos(t2)**2*math.cos(t3)**2 + math.sin(t3)**2))


    return np.array([x, y, z, alpha, beta, gamma])
