import scipy.io as sio
import os
import matplotlib.pyplot as plt
import numpy as np
import scipy as sp
import skrf as rf
import copy
import time

def mergeVectorsToSparam(vs11, vs12, vs21, vs22):
    ss = []
    for s11, s12, s21, s22 in zip(vs11, vs12, vs21, vs22):
        s=np.array([[s11, s12], [s21, s22]])
        ss.append(s)
    return np.stack(ss)


def createNetwork(f, S21, S11=None, S12=None, S22=None):
    if S12 is None:
        S12=S21
    if S11 is None:
        S11=np.sqrt(1-S21**2)
    if S22 is None:
        S22=np.sqrt(1-S12**2)
    return rf.network.Network(f=f, s=mergeVectorsToSparam(S11, S12, S21, S22), z0=[50, 50], f_unit='Hz')

def vTodB(v):
    return 20*np.log10(np.abs(v))

class SimpleS21Fixture():
    def __init__(self, f, s21):
        self.de=sp.interpolate.interp1d(f, s21)
        
    def deembedFrom(self, f, s21):
        return s21/self.de(f)
    

class YigFilterSimulation:
    def __init__(self, modelDataPath, deembed):
        self.fileName=modelDataPath
        deembedData=sio.loadmat(deembed)
        self.deembed=SimpleS21Fixture(np.squeeze(deembedData['f']),np.squeeze(deembedData['S21']))
        self.filterModelData=sio.loadmat(modelDataPath)
        self.trimFilterModelData()
        self.filterMatrix=self.createFilterMatrix()
        self.frequencyMatrix=self.filterModelData['filterFrequency']
        self.current=self.filterModelData['filterCurrent']       
        
    def getFrequencyRange(self):
        return (np.median(self.frequencyMatrix[0,:]), np.median(self.frequencyMatrix[-1,-1]))
        
    def plotPassBands(self):
        plt.figure()
        plt.imshow(20*np.log10(np.abs(self.filterMatrix)), aspect='auto')
        plt.colorbar()
  
    def createFilterMatrix(self):
        filterMatrix=[]
        for S21, f in zip(self.filterModelData['filterS21'], self.filterModelData['filterFrequency']):
            dembeddedS21 = self.deembed.deembedFrom(f, S21)
            filterMatrix.append(dembeddedS21)
        return np.array(filterMatrix)
            
    def getFilterS21Unbiased(self):
        f=self.filterModelData['frequencies_wide_no_current'][0]
        s21=self.filterModelData['s21_wide_no_current'][0]
        deembedded=self.deembed.deembedFrom(f,s21)
        return f, deembedded
    
    def getFilterPeakResponse(self, current):
        idx=np.abs(self.current-current).argmin()
        nc=self.current[idx]
        if nc > current:
            hidx=idx
            lidx=idx-1
        else:
            hidx=idx+1
            lidx=idx
        lc=self.current[lidx]    
        hc=self.current[hidx]
        dc=hc-lc
        hw=(current-lc)/dc
        lw=1-hw
        f = self.frequencyMatrix[lidx,:]*lw+self.frequencyMatrix[hidx,:]*hw
        smag = np.abs(self.filterMatrix[lidx,:])*lw+np.abs(self.filterMatrix[hidx,:])*hw
        spha = np.angle(self.filterMatrix[lidx,:])*lw+np.angle(self.filterMatrix[hidx,:])*hw
        s=smag*np.exp(1j*spha)
        return f,s
    
    
    def getFilterResponse(self, current):
        pf, ps = self.getFilterPeakResponse(current)
        tf, ts = self.getFilterS21Unbiased()
        lowUnbiased=np.where(tf<pf[0])
        highUnbiased=np.where(tf>pf[-1])
        f=np.concatenate([tf[lowUnbiased], pf, tf[highUnbiased]])
        s=np.concatenate([ts[lowUnbiased], ps, ts[highUnbiased]])
        return f,s    

    def plotModel(self):
        plt.figure()
        fmin,fmax=self.getFrequencyRange()
        plt.title(f"{self.fileName}, sweep from {fmin/1e6} to {fmax/1e6} MHz")
        num = len(self.current)
        for i in range(num):
            thisCurrent=self.current[i]
            
            if i == num-1:
                thisCurrent-=0.00001
            if i == 0:
                thisCurrent+=0.00001
            f,s=self.getFilterResponse(thisCurrent)
            plt.plot(f,vTodB(s),linewidth=3.5)
            if i < num -1:
                nextCurrent=self.current[i+1]
                f,s=self.getFilterResponse((thisCurrent+nextCurrent)/2)
                plt.plot(f,vTodB(s), linewidth=0.5)
        plt.grid()
    
    def trimFilterModelData(self):
        c=self.filterModelData['filterCurrent'][0]
        cd=np.diff(c)
        numSigma=2
        ub = np.median(cd)+np.std(cd)*numSigma
        lb = np.median(cd)-np.std(cd)*numSigma
        validIdx=np.where( (cd>=lb) & (cd <= ub))
        #print(self.filterModelData['filterCurrent'].shape)
        #print(self.filterModelData['filterS21'].shape)
        #print(self.filterModelData['filterFrequency'].shape)
        self.filterModelData['filterCurrent']=np.squeeze(self.filterModelData['filterCurrent'][:,validIdx])
        self.filterModelData['filterS21']=np.squeeze(self.filterModelData['filterS21'][validIdx,:])
        self.filterModelData['filterFrequency']=np.squeeze(self.filterModelData['filterFrequency'][validIdx,:])
        #print(self.filterModelData['filterCurrent'].shape)
        #print(self.filterModelData['filterS21'].shape)
        #print(self.filterModelData['filterFrequency'].shape)
