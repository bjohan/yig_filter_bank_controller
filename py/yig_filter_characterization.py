import matplotlib.pyplot as plt
import numpy as np
import scipy as sp
import skrf as rf


import yig_filter_model as yfm


class YigCharacterization(yfm.YigFilterSimulation):
    def __init__(self, modelFile, fixtureFile):
        super().__init__(modelFile, fixtureFile)
        self.slope, self.zeroF = self.computeFilterParameters()
    
    def getMaxPeak(self, f, s21):
        return f[np.abs(s21).argmax()]
        
    def getCenterMass(self, f, s21):
        sm = np.sum(np.abs(s21))
        tq = np.sum(np.multiply(np.abs(s21),f))
        cog=tq/sm;
        #print("cog", cog, "sm", sm, "tq", tq)
        return cog
    
    def getCenterMassSquare(self, f, s21):
        power=s21**2
        sm = np.sum(np.abs(power))
        tq = np.sum(np.multiply(np.abs(power),f))
        cog=tq/sm;
        #print("cog", cog, "sm", sm, "tq", tq)
        return cog
        
    def getThresholdCenter(self, f, s21, threshold):  
        level=np.max(np.abs(s21))*10**(threshold/20.0)
        idxs=np.where(np.abs(s21)>level)
        return self.getCenterMassSquare(f[idxs], s21[idxs])
        #mini=np.min(idxs)
        #maxi=np.max(idxs)
        #return (f[mini]+f[maxi])/2
        
        
    def analyzeResponses(self):
        maxPeak=[]
        centerMass=[]
        centerMassSquare=[]
        currents=[]
        centerTheshold10Db=[]
        for i in range(len(self.current)):
            s21=self.filterMatrix[i,:]
            if np.max(np.abs(s21))>1/3.0:
                f=self.frequencyMatrix[i,:]
                maxPeak.append(self.getMaxPeak(f, s21))
                centerMass.append(self.getCenterMass(f,s21))
                centerMassSquare.append(self.getCenterMassSquare(f,s21))
                centerTheshold10Db.append(self.getThresholdCenter(f, s21, -10))
                currents.append(self.current[i])
                

        maxPeak=np.array(maxPeak)
        centerMass=np.array(centerMass)
        centerMassSquare=np.array(centerMassSquare)
        centerTheshold10Db=np.array(centerTheshold10Db)
        currents=np.array(currents)
        return maxPeak, centerMass, centerMassSquare, centerTheshold10Db, currents
    
    def analyze(self):
        maxPeak, centerMass, centerMassSquare, centerTheshold10Db, currents=self.analyzeResponses()        
        plt.figure()
        plt.plot(currents[1:], np.diff(maxPeak/1e6)/np.diff(currents*1e3))
        plt.plot(currents[1:], np.diff(centerMass/1e6)/np.diff(currents*1e3))
        plt.plot(currents[1:], np.diff(centerMassSquare/1e6)/np.diff(currents*1e3))
        plt.plot(currents[1:], np.diff(centerTheshold10Db/1e6)/np.diff(currents*1e3))
        plt.xlabel('Current [A]')
        plt.ylabel('Slope [MHz/mA]')
        plt.legend(["max", "center mass voltage", "center mass power", '10dB', "6dB", "3dB"])
        plt.grid(True)

        
    def computeSlope(self, cm, currents):
        slope=np.diff(cm)/np.diff(currents)
        slopeDev=np.std(slope)
        slopemedian=np.median(slope)
        lb = slopemedian-slopeDev
        ub = slopemedian+slopeDev
        a = slope[np.where(slope>lb)]
        vld = slope[np.where(a<ub)]
        return np.mean(vld)
    
    
    def computeFilterParameterVectors(self, cm, currents):
        slope=self.computeSlope(cm, currents)
        zeroF = cm-currents*slope
        dcCurrent=-zeroF/slope
        return slope, zeroF, dcCurrent
        
        
    def plotFilterParameters(self):
        maxPeak, centerMass, centerMassSquare, centerTheshold10Db, currents=self.analyzeResponses()   
        
        slope, zeroF, dcCurrent=self.computeFilterParameterVectors(centerMass, currents)
       
        plt.figure()
        plt.plot(currents, zeroF/1e6)
        plt.title('Zero current frequency, frequency where current is 0')
        plt.xlabel('current [A]')
        plt.ylabel('Zero current frequency [MHz]')
        plt.grid(True)
        
       
        plt.figure()
        plt.plot(currents, dcCurrent/1e6)
        plt.title('DC Current, current where filter would pass DC')
        plt.xlabel('current [A]')
        plt.ylabel('DC current [A]')
        plt.grid(True)
        
    def computeFilterParameters(self):
        maxPeak, centerMass, centerMassSquare, centerTheshold10Db, currents=self.analyzeResponses()   
        slope, zeroF, dcCurrent=self.computeFilterParameterVectors(centerMass, currents)
        return np.mean(slope), np.mean(zeroF)
        
    def computeTuningCurrent(self, f):
        return (f-self.zeroF)/self.slope

    def currentToFrequency(self, i):
        return i*self.slope+self.zeroF

    def currentsToFrequencies(self, currents):
        fs = []
        for i in currents:
            fs.append(self.currentToFrequency(i))
        return fs
    
    def getTunedResponse(self, f):
        current = self.computeTuningCurrent(f)
        return self.getFilterResponse(current)
