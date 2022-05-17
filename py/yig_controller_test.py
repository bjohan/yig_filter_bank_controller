import sys
sys.path.append("gpib_instrument_control")
import numpy as np
import matplotlib.pyplot as plt
import hp_3478a
import tqdm
import serial
import time
import scipy.io as sio
import hp_8700_series_vna
import hp_3478a
import argparse
import skrf.network
import scipy.interpolate
import random

class SerialWrap:
    def __init__(self,ser):
        self.ser=ser

    def write(self, data):
        print("Writing=>",data.decode('ascii'));
        return self.ser.write(data)

    def read(self, **kwargs):
        d=self.ser.read(**kwargs)
        if len(d):
            print("Read<=", d.decode('ascii'))
        else:
            print('.', end='')
        return d

class Switch:
    def __init__(self, dev, designation):
        self.dev=dev;
        self.designation=designation;
        self.state=None

    def set(self, state):
        if state not in range(7):
            raise ValueError("Illegal switch state must be 0..6")
        self.dev.write(b"SWITCH %s %d\n"%(self.designation, state))

    def __str__(self):
        return "Switch %s state %d"%(self.designation.decode('ascii'), self.state)

    def parseStatusLine(self, line):
        if b'Relay' in line:
            v=int(line.split(b':')[-1])
            if self.designation in line:
                self.state=v;
                return True
        return False

class YigChannel:
    def __init__(self, dev, designation):
        self.dev=dev
        self.designation=designation
        self.outputChannel=None
        self.outputValue=None
    
    def __str__(self):
        return "Yig channel %s driving %d with value %d"%(self.designation.decode('ascii'), self.outputChannel, self.outputValue)

    def set(self, channel, value):
        if channel not in range(8):
            raise ValueError("Illegal switch state must be 0..5")
        if value not in range(-32768, 32768):
            raise ValueError("Illegal value %d, must be -32768..32767"%(value))
        self.dev.write(b"YIG %s %d %d\n"%(self.designation, channel, value))

    def writeFilterSlope(self, i, f):
        if i not in range(8):
            raise ValueError("Illegal filter index, must be 0..7")
        self.dev.write(b"COEF %s S %d %f\n"%(self.designation, i, f))

    def writeFilterOffset(self, i, offset):
        if i not in range(8):
            raise ValueError("Illegal filter index, must be 0..7")
        #if offset not in range(-32768, 32768):
        #    raise ValueError("Illegal offset %d, must be -32768..32767"%(value))
        self.dev.write(b"COEF %s O %d %f\n"%(self.designation, i, offset))

    def writeFilterLowLim(self, i, f):
        if i not in range(8):
            raise ValueError("Illegal filter index, must be 0..7")
        self.dev.write(b"COEF %s L %d %f\n"%(self.designation, i, f))

    def writeFilterHighLim(self, i, f):
        if i not in range(8):
            raise ValueError("Illegal filter index, must be 0..7")
        self.dev.write(b"COEF %s H %d %f\n"%(self.designation, i, f))

    def save(self):
        self.dev.write(b"M S\n")

    def load(self):
        self.dev.write(b"M L\n")

    def parseStatusLine(self, line):
        if b'YIG' in line:
            toks=line.split(b' ')
            if toks [2] == self.designation:
                self.outputChannel=int(toks[4])
                self.outputValue=int(toks[6])
                return True;
        return False

class YigController:
    def __init__(self, dev):
        #self.dev=SerialWrap(serial.Serial(dev, 115200, timeout=1));
        self.dev=serial.Serial(dev, 115200, timeout=1);
        self.waitForBoot()
        self.switchA=Switch(self.dev, b'A')
        self.switchB=Switch(self.dev, b'B')
        self.yigA=YigChannel(self.dev , b'A')
        self.yigB=YigChannel(self.dev , b'B')
        self.updateStatus()

    def __str__(self):
        return "%s %s %s %s"%(self.switchA, self.switchB, self.yigA, self.yigB)

    def waitForBoot(self):
        print("Waiting for init... ", end='')
        sys.stdout.flush();
        r=b''
        while True:
            d=self.dev.read()
            r+=d;
            if b'INIT DONE\r\n' in r:
                break
        print('Done')


    def parseStatusLine(self, line):
        parsed=False;
        parsed |= self.switchA.parseStatusLine(line)
        parsed |= self.switchB.parseStatusLine(line)
        parsed |= self.yigA.parseStatusLine(line)
        parsed |= self.yigB.parseStatusLine(line)
        if not parsed:
            print("Warning unparsed line")
        return parsed

    def updateStatus(self):
        self.dev.write(b'S\n');
        r=b''
        
        while True:
            d=self.dev.read()
            if len(d)==0:
                print('timeout whil reading', r.decode('ascii'))
                break
            #print(d);
            r+=d;
            if r.count(b'\n')==4:
                if b'YIG driver B' in r:
                    break
        for l in r.splitlines():
            if not self.parseStatusLine(l):
                print(l)


class YigFilter:
    def __init__(self, flow, fhigh, offset, slope, yigController, num, new=False):
        self.flow=flow
        self.fhigh=fhigh
        self.offset=offset
        self.slope=slope
        self.yc=yigController
        self.num=num
        self.new=new

    def computetTuningWord(self, f):
        if self.new:
            return int(f*self.slope+self.offset)
        else:
            return int((f-self.offset)/self.slope)

    def computeSlopeLimits(self):
        if self.new:
            raise Exception("Implement if needed")
            pass
        else:
            ma=-32768*self.slope+self.offset
            mi=32767*self.slope+self.offset
            return (mi, ma)

    def setSwitch(self):
        self.yc.switchA.set(self.num+1)

    def tuneTo(self, f, channel = None):
        self.setSwitch()
        if channel is None:
            self.yc.yigA.set(self.num, self.computetTuningWord(f))
        else:
            channel.set(self.num, self.computetTuningWord(f))

    def finRange(self, f):
        if f > self.flow and f < self.fhigh:
            return True
        return False

class YigFilterBank:
    def __init__(self, yigController):
         self.filters=[
                YigFilter(0.5e9, 1e9,  915698639.5899, -28587.5706, yigController, 0),
                YigFilter(0.5e9, 1e9,  2149392744.4795, -72171.4547, yigController, 1),
                YigFilter(0.5e9, 1e9,  3216433121.0191, -110012.6209, yigController, 2),
                YigFilter(0.5e9, 1e9,  5516742307.6923, -190038.8918, yigController, 3),
                YigFilter(0.5e9, 1e9,  9097690625, -312743.852, yigController, 4),
                YigFilter(0.5e9, 1e9,  12028714843.75, -410176.4825, yigController, 5)]

    def tuneTo(self, f):
        for filt in self.filters:
            if filt.finRange(f):
                filt.tuneTo(f)
                return
        raise ValueError("Frequency %d is not within the range of this filter bank"%(f))
        
class YigTuner:
    def __init__(self, yigController):
        self.yc=yigController;


    def tuneTo(self, f):
        if f < 0.5e9:
            return;
        if f<1e9:
            self.yc.switchA.set(1)
            o=928241560;
            s=-29942.3333;
            o=915698639.5899
            s=-28587.5706
            self.yc.yigA.set(0, int((f-o)/s))
            return;
        if f<2e9:
            self.yc.switchA.set(2)
            o=2156219920;
            s=-71735.7083;
            o=2149392744.4795
            s=-72171.4547
            #f=f+18e6
            self.yc.yigA.set(1, int((f-o)/s))
            return;
        if f<4e9:
            self.yc.switchA.set(3)
            o=3222441710;
            s=-110335.9792;
            o=3216433121.0191
            s=-110012.6209
            self.yc.yigA.set(2, int((f-o)/s))
            return;
        if f<8e9:
            self.yc.switchA.set(4)
            o=5518319170;
            s=-189787.6907;
            o=5516742307.6923
            s=-190038.8918
            self.yc.yigA.set(3, int((f-o)/s))
            return;
        if f<12e9:
            self.yc.switchA.set(5)
            o=9107832094;
            s=-310683.7808;
            o=9097690625
            s=-312743.852
            self.yc.yigA.set(4, int((f-o)/s))
            return;
        if f<18e9:
            self.yc.switchA.set(6)
            o=12041133605;
            s=-411474.2218;
            o=12028714843.75
            s=-410176.4825
            self.yc.yigA.set(5, int((f-o)/s))
            return;


def stackVector(mtx, vec):
    if mtx is None:
        mtx=np.array(((vec)))
    else:
        mtx=np.row_stack((mtx, vec))
    return mtx


def createFilterModel(filePath, numPoints, filterIdx, span=700e6):
        dataDict={}
        meter=hp_3478a.Hp3478A()
        vna=hp_8700_series_vna.Hp8753A()
        yc=YigController('/dev/ttyUSB0')
        yfb=YigFilterBank(yc)
        filterToMeasure=yfb.filters[filterIdx]
        fstart, fstop=filterToMeasure.computeSlopeLimits()
        print("Yig driver theoretical range from %d MHz to %d MHz"%(fstart/1e6, fstop/1e6))
        print("5s settling time before wide band measurement")
        yfb.filters[filterIdx].setSwitch()
        time.sleep(10)
        vna.reset()
        vna.setPoints(1601)
        dataDict['s21_wide_no_current']=vna.readSParameter('S21');
        dataDict['frequencies_wide_no_current']=vna.frequencies();
    
        if fstart-span/2 < 130e6:
            fstartNew = 130e6+span/2
            print("Adjusted fstart from %d MHz to %d Mhz due to vna limitations"%(fstart/1e6, fstartNew/1e6))
            fstart = fstartNew

        if fstop+span/2 > 20e9:
            fstopNew=20e9-span/2
            print("Adjusted fstop from %d MHz to %d Mhz due to vna limitations"%(fstop/1e6, fstopNew/1e6))
            fstop=fstopNew

        sweep=np.linspace(fstart, fstop, numPoints)
        print("Sweeping from %d MHz to %d MHz in %d steps"%(fstart/1e6, fstop/1e6, numPoints))
        filterData=None
        frequencyData=None
        currentData=[]
        for f in sweep:
            filterToMeasure.tuneTo(f)
            vna.setStartFrequency(f-span/2);
            vna.setStopFrequency(f+span/2)
            time.sleep(1)
            filterData=stackVector(filterData, vna.readSParameter('S21'))
            currentData.append(meter.readValue())
            frequencyData=stackVector(frequencyData, vna.frequencies())
            
        dataDict['filterS21']=filterData
        dataDict['filterFrequency']=frequencyData
        dataDict['filterCurrent']=currentData
        sio.savemat(filePath, dataDict)
 
if __name__ == "__main__":
    parser = argparse.ArgumentParser("Tool for measuring on yig controller circuit");
    parser.add_argument('-s', '--sweep', help="Measure s parameter while sweeping control word", action='store_true');
    parser.add_argument('-td', '--temperature-drift', help="Measure temperature drift", action='store_true')
    parser.add_argument('-tc', '--temperature-current', help="Measure temperature current drift", action='store_true')
    parser.add_argument('-ts', '--tune-sweep', help='Measure a tuned sweep', action='store_true')
    parser.add_argument('-r', '--random', help='Go through the tune sweep in random order', action='store_true')
    parser.add_argument('-n', '--name', help='Append this to filename')
    parser.add_argument('-t', '--toggle-min-max', help='Toggle indicated filter between min and max current', type=int)
    parser.add_argument('-ycw', '--yig-control-word', help='write yig control word to set current', type=int)
    parser.add_argument('-cfm', '--create-filter-model', help='Create filter model, use --filter-number and --number-of-points to specify how the model should be created')
    parser.add_argument('-np', '--number-of-points', help='Number of points to sweep', type=int)
    parser.add_argument('-fn', '--filter-number', help='Index of filter in bank', type=int)
    parser.add_argument('-myc', '--measure-yig-coefficients', help="Measure coefficients of yig filter", type=int); 

    pa=parser.parse_args()


    if pa.create_filter_model:
        createFilterModel(pa.create_filter_model, pa.number_of_points, pa.filter_number) 

        quit()

        




    extraName=''
    if pa.name:
        extraName='_'+pa.name



    yc=YigController('/dev/ttyUSB0')
    yc.updateStatus()
    yc.switchA.set(6);
    yt=YigTuner(yc)
    time.sleep(0.1);



    widePoints=3;
    narrowPoints=128;
    measureFrequencies={
            1:np.linspace(130e6, 2e9, 64).astype(int),
            2:np.linspace(500e6, 4.5e9, 64).astype(int),
            3:np.linspace(500e6, 6.5e9, 128).astype(int),
            4:np.linspace(1e6, 10e9, 256).astype(int),
            5:np.linspace(1e6, 16.5e9, 384).astype(int),
            6:np.linspace(3e6, 20e9, 384).astype(int),
            }



    if pa.measure_yig_coefficients is not None:
        import scipy.signal
        vna=hp_8700_series_vna.Hp8753A()
        meter=hp_3478a.Hp3478A()
        vna.setPoints(201)
        ch=pa.measure_yig_coefficients
        och=yc.yigA.outputChannel
        ocv=yc.yigA.outputValue
        yc.switchA.set(ch+1)
        yc.yigA.set(ch,-32768)
        vna.setStartFrequency(130e6)
        vna.setStopFrequency(20e9)
        s21Low=vna.readSParameter('S21');
        yc.yigA.set(ch, 32767)
        s21High=vna.readSParameter('S21')
        f=vna.frequencies()
        yc.yigA.set(och, ocv)
        plt.plot(f, 20*np.log10(abs(s21Low)))
        plt.plot(f, 20*np.log10(abs(s21High)))
        diff=np.abs(20*np.log10(abs(s21Low/s21High)));
        plt.plot(f, diff)
        plt.pause(0.2)
        pidx=scipy.signal.find_peaks(diff, height=5)[0]
        peakFreqs=np.array(f)[pidx]
        minFreq=np.min(peakFreqs)
        minFreq=0
        maxFreq=np.max(peakFreqs)
        print("Filter frequency %.1f MHz to %.1f MHz"%(minFreq/1e6, maxFreq/1e6))
        margin=200e6
        fstart=minFreq-margin
        fstop=maxFreq+margin
        if fstart < 130e6:
            fstart=130e6
        if fstop > 20e9:
            fstop=20e9
        time.sleep(5)
        vna.setStartFrequency(fstart)
        vna.setStopFrequency(fstop)
        vna.setPoints(1601)
        f=vna.frequencies()
        base=vna.readSParameter('S21')
        words=np.linspace(-32768, 32767, 20)
        plt.figure(1)
        plt.clf()
        plt.plot(f, 20*np.log10(np.abs(base)))
        current=[]
        currentw=[]
        for w in words:
            plt.figure(1)
            yc.yigA.set(ch, int(w))
            s21=vna.readSParameter('S21');
            current.append(meter.readValue())
            currentw.append(w)
            diff=20*np.log10(abs(s21))
            plt.plot(f, diff)
            plt.pause(0.2)
            plt.figure(2)
            plt.plot(currentw, current)
            plt.pause(0.2)
        print("done")    
        plt.show()
        yc.yigA.set(och, ocv)


        

    if pa.toggle_min_max is not None:
        fc=pa.toggle_min_max
        print("toggling", fc)
        while True:
            yc.yigA.set(fc, 32767)
            time.sleep(1);
            yc.yigA.set(fc, 0)
            time.sleep(1)
            quit(0)

    if pa.temperature_current:
        meter=hp_3478a.Hp3478A()
        totalTime=60*5;
        saveDict={};
        if pa.yig_control_word is not None:
            w=pa.yig_control_word
        else:
            print("Specify --yig-control-word to to set current")

        for i in tqdm.tqdm(range(6)):
            times=[]
            currents=[]
            yc.yigA.set(i, w)
            t0=time.time();
            tp=np.linspace(0, totalTime, int(totalTime/2))+t0;
            for t in tqdm.tqdm(tp):
                while time.time() < t:
                    currents.append(meter.readValue())
                    times.append(time.time()-t0)
            saveDict["yig_%d_current"%(i)]=currents;
            saveDict["yig_%d_time"%(i)]=times;
        saveDict["control_word"]=w
        sio.savemat('yig_temp_current%s.mat'%(extraName), saveDict)



#add filter model option
#first measure unbiased filter
#fig

    if pa.tune_sweep:
        vna=hp_8700_series_vna.Hp8753A()
        meter=hp_3478a.Hp3478A()
        filterData=None
        numpt=64*16;
        sweepFrequencies=np.linspace(500e6, 18e9, numpt);
        print(sweepFrequencies)
        if pa.random:
            random.shuffle(sweepFrequencies)
            extraName+='_random'
        vna.reset()
        vna.setPoints(201)
        span=500e6;
        freqData=None
        currentData=[]
        currentDataPre=[]
        for mf in tqdm.tqdm(sweepFrequencies):
            yt.tuneTo(mf)
            time.sleep(1)
            currentDataPre.append(meter.readValue())
            vna.setStartFrequency(mf-span/2);
            vna.setStopFrequency(mf+span/2)
            time.sleep(0.1);
            s21=vna.readSParameter('S21');
            if filterData is None:
                filterData=np.array(((s21)))
            else:
                filterData=np.row_stack((filterData, s21))
            if freqData is None:
                freqData=np.array((vna.frequencies()))
            else:
                freqData=np.row_stack((freqData, vna.frequencies()))
            currentData.append(meter.readValue())


        currentData=np.array(currentData)
        currentDataPre=np.array(currentDataPre)
        sio.savemat('yig_tune_sweep%d%s.mat'%(numpt,extraName), {'sweptFrequencies':sweepFrequencies, 'filterData':filterData, 'freqData': freqData, 'current':currentData, 'currentPre':currentDataPre})

    if pa.sweep:
        vna=hp_8700_series_vna.Hp8753A()
        meter=hp_3478a.Hp3478A()
        baseCal=skrf.network.Network(file='cables.s2p')
        #print(dir(baseCal))
        #print(baseCal)
        #print(dir(baseCal.frequency))
        #print(dir(baseCal.frequency.f))
        #print(baseCal.frequency.f)
        #print(dir(baseCal.s21))
        #print(dir(baseCal[2,1]))
        baseCalZ=np.squeeze(baseCal[2,1].z)
        #baseCal.plot_s_db()
        calFunc = scipy.interpolate.interp1d(baseCal.frequency.f/1e9, np.abs(baseCalZ))
        if pa.random:
            extraName+='_random'

        for mf in tqdm.tqdm(measureFrequencies):
            vna.reset()
            vna.setPoints(1601)
            fwide=vna.frequencies()
            #s21cal=calFunc(fwide)
            #s21cal=vna.readSParameter('S21');
            filterSel=mf

            yc.switchA.set(filterSel);
            values=np.linspace(-32768, 32767, widePoints).astype(int)
            filterData=None
            for v in tqdm.tqdm(values):
                yc.yigA.set(filterSel-1, v);
                time.sleep(0.1);
                s21=vna.readSParameter('S21');
                #s21=s21/s21cal
                #plt.plot(abs(s21))
                #plt.show()
                if filterData is None:
                    filterData=np.array(((s21)))
                else:
                    filterData=np.row_stack((filterData, s21))
                #print(filterData.shape)
            filterWide=filterData;
            vwide=values;
            #calwide=s21cal

            vna.setPoints(1601)
            ffreqs=measureFrequencies[mf]
            vna.setStartFrequency(ffreqs[0])
            vna.setStopFrequency(ffreqs[-1])
            yc.switchA.set(6);
            time.sleep(0.1);
            #s21cal=vna.readSParameter('S21');
            yc.switchA.set(filterSel);
            time.sleep(0.1);
            f=vna.frequencies()
            #s21cal=calFunc(f)
            filterData=None
            currentData=[]
            currentDataPre=[]
            values=np.linspace(-32768, 32767, len(measureFrequencies[mf])).astype(int)
            values=np.linspace(-15000, 25000, len(measureFrequencies[mf])).astype(int)
            if pa.random:
                random.shuffle(values)
            for v in tqdm.tqdm(values):
                yc.yigA.set(filterSel-1, int(v));
                time.sleep(4);
                currentDataPre.append(meter.readValue())
                s21=vna.readSParameter('S21');
                #s21=s21/s21cal
                if filterData is None:
                    filterData=np.array(((s21)))
                else:
                    filterData=np.row_stack((filterData, s21))
                currentData.append(meter.readValue())
                #print(filterData.shape)

            sio.savemat('yigtest%d%s.mat'%(filterSel, extraName), {'fwide':fwide, 'filterWide':filterWide, 'vwide': vwide, 'f':f, 'filter':filterData, 'v':values, 'current':currentData, 'currentPre':currentDataPre})

    if pa.temperature_drift:
        vna=hp_8700_series_vna.Hp8753A()
        print("Measuring temperature drift")
        filterSel=2;
        yc.switchA.set(6)
        vna.reset()
        vna.setPoints(1601)
        s21cal=vna.readSParameter('S21');
        yc.switchA.set(filterSel);
        yc.yigA.set(filterSel-1, -15000)
        yc.yigA.set(filterSel-1, 10000)
        S21=vna.readSParameter('S21')
        mi=np.argmax(S21)
        mf=vna.frequencies()[mi];
        vna.setStartFrequency(mf-150e6);
        vna.setStopFrequency(mf+150e6);
        driftData=None
        timeData=[]
        t0=time.time();
        toSave=0
        try:
            fax=vna.frequencies()
            while True:
                    S21=vna.readSParameter('S21')
                    S21=S21/s21cal
                    if driftData is None:
                        driftData=np.array(((S21)))
                    else:
                        driftData=np.row_stack((driftData, S21))
    
                    mi=np.argmax(S21)
                    elapsed=time.time()-t0
                    timeData.append(elapsed)
                    print("Peak frequency", vna.frequencies()[mi], "after", elapsed, "seconds");
                    if toSave == 0:
                        print("Saving data")
                        sio.savemat('yig_temperature_low_current_drift%d_running%d%s.mat'%(filterSel, extraName), {'timeData':timeData, 'frequencyData':fax, 'driftData':driftData});
                        toSave=20
                    else:
                        toSave-=1

                    if elapsed > 1000:
                        break
                    #time.sleep(20)

        except KeyboardInterrupt:
            pass
        print("Saving data")
        sio.savemat('yig_temperature_low_current_drift%d%s.mat'%(filterSel, extraName), {'timeData':timeData, 'frequencyData':fax, 'driftData':driftData});
