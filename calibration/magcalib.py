import csv
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

import os
import sys
import numpy as np
import json

# Ajoute le dossier parent (racine du projet) au chemin de recherche de Python
dossier_parent = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(dossier_parent)

from drivers.ddboatlib import drawEllipsoid


def Fit_Ellipsoid(x1,x2,x3):
    x1,x2,x3=np.array(x1),np.array(x2),np.array(x3)
    X = np.column_stack([x1**2,x2**2,x3**2,x1*x2,x1*x3,x2*x3,x1,x2,x3])
    p = inv(X.T@X)@X.T @ np.ones((len(x1), 1))
    p=p.flatten()
    P = np.array([[p[0],1/2*p[3],1/2*p[4]],[1/2*p[3], p[1],1/2*p[5]],[1/2*p[4], 1/2*p[5], p[2]]])
    c=np.array([[p[6]],[p[7]],[p[8]]])
    xbar=-1/2*inv(P)@c
    Q = P/(1+xbar.T@P@xbar)
    print("The matrix Q should be symmetric: \n Q=",Q)
    print("The eigen values of Q should be positive : \n eig(Q)=",eig(Q)[0])
    return Q,xbar



def mag_calib(x1,x2,x3,xnorth,xwest):
    Q,xbar=Fit_Ellipsoid(x1,x2,x3)
    I=64*3.14/180
    ynorth=np.array([[np.cos(I)],[0],[-np.sin(I)]]) # What I want when the boat points to North
    ywest=np.array([[0],[-np.cos(I)],[-np.sin(I)]]) # What I want when the boat points to West
    yup=np.cross(ynorth.flatten(), ywest.flatten()).reshape(3,1)
    Y=np.hstack([ynorth,ywest,yup])
    vnorth=sqrtm(Q)@(xnorth-xbar)
    vwest=sqrtm(Q)@(xwest-xbar)
    print("Should be equal to 1 --> norm(vnorth),norm(vwest)=",norm(vnorth),norm(vwest))
    vup=np.cross(vnorth.flatten(),vwest.flatten()).reshape(3,1)
    V=np.hstack([vnorth,vwest,vup])
    R=Y@inv(V)
    print("Should be a rotation -> R=",R)
    print("R@R.T",R@R.T)
    invA=R@sqrtm(Q)
    return invA,xbar


def save_calibration_json(filepath, invA, xbar):
    """Sauvegarde invA et xbar dans un fichier JSON."""
    data = {
        "invA": invA.tolist(),
        "xbar": xbar.tolist()
    }
    with open(filepath, 'w') as f:
        json.dump(data, f, indent=4)
    #print(f"Matrices d'étalonnage sauvegardées dans '{filepath}'")

def load_calibration_json(filepath="mag_calib_params.json"):
    """
    Lit le fichier JSON d'étalonnage et renvoie invA et xbar sous forme de np.ndarray.
    """
    with open(filepath, 'r') as f:
        data = json.load(f)
    
    invA = np.array(data["invA"], dtype=float)
    xbar = np.array(data["xbar"], dtype=float)
    return invA, xbar



if __name__ == "__main__" :
    from scipy.linalg import sqrtm,inv,eig, norm
    
    x1,x2,x3 = np.loadtxt('magcloud.csv', delimiter=',', unpack=True)

    fig = plt.figure(figsize=(8,8))
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(x1,x2,x3,c="blue")
    plt.pause(0.1)

    data_north = np.loadtxt('x_north.csv', delimiter=',')
    data_west = np.loadtxt('x_west.csv', delimiter=',')
    xnorth = np.mean(data_north, axis=0).reshape(3, 1)
    xwest = np.mean(data_west, axis=0).reshape(3, 1)

    print("xnorth:", xnorth)
    print("xwest:", xwest)

    ax.scatter(*xnorth[:,0],c="red")
    ax.scatter(*xwest[:,0],c="red")

    invA,xbar=mag_calib(x1,x2,x3,xnorth,xwest)
    print("Ahat= ",inv(invA))
    print("xbar=",xbar)

    save_calibration_json("mag_calib_params.json", invA, xbar)

    drawEllipsoid(ax,xbar,invA.T@invA)

    plt.show()


