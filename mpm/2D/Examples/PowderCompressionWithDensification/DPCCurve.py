import numpy as np
import matplotlib.pyplot as plt

beta = 1.221
R = 0.56
d = 0.295 
Pb = 3.345e3
Pa = (Pb - R*d)/(1+R*np.tan(beta))

x = np.linspace(Pa,Pb,200)
y = np.sqrt((d+Pa*np.tan(beta))**2-((x-Pa)/R)**2)

x0 = np.array([0])
y0 = np.array([d])
xf = np.concatenate((x0,x),axis=None)
yf = np.concatenate((y0,y),axis=None)

with open("/home/ab287450/Codes/mpmxdem/mpm/2D/Examples/PowderCompression/DPCCurve.txt","w") as text_file :
    # l1 = 
    text_file.write(f"# Courbe DPC beta = {beta}, R = {R}, d = {d}, Pb = {Pb} \n")
    text_file.write(f"0 {d} \n")
    for i in range(200):
            text_file.write(f"{x[i]} {y[i]}\n")
