#!/usr/bin/python3
import serial
import sys


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
        self.dev.write(b"COEF %s S %d %.15f\n"%(self.designation, i, f))

    def writeFilterOffset(self, i, offset):
        if i not in range(8):
            raise ValueError("Illegal filter index, must be 0..7")
        #if offset not in range(-32768, 32768):
        #    raise ValueError("Illegal offset %d, must be -32768..32767"%(value))
        self.dev.write(b"COEF %s O %d %.10f\n"%(self.designation, i, offset))

    def writeFilterLowLim(self, i, f):
        if i not in range(8):
            raise ValueError("Illegal filter index, must be 0..7")
        self.dev.write(b"COEF %s L %d %.5f\n"%(self.designation, i, f))

    def writeFilterHighLim(self, i, f):
        if i not in range(8):
            raise ValueError("Illegal filter index, must be 0..7")
        self.dev.write(b"COEF %s H %d %.5f\n"%(self.designation, i, f))

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

    def tune(self, f):
        self.dev.write(b"T A %.2f\n"%(f))

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



if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser("Small tool for operating yig filter bank from the command line")
    parser.add_argument("port", help="Serial port to use for connecting to yig filter bank")
    parser.add_argument("-t", "--tune", help="Tune filter to this frequency", metavar="FREQ", type=float)

    args = parser.parse_args()

    yc = YigController(args.port)
    if args.tune:
        yc.tune(args.tune)
        print("Tuned to %0.2f GHz"%(args.tune/1e9))
