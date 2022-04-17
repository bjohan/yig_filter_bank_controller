import yig_filter_characterization as yfcm
class YigFilterBank:
    def __init__(self, path):
        self.prefilters=[]
        self.postfilters=[]
        for i in range(6):
            self.prefilters.append(yfcm.YigCharacterization(path%("pre", i), './yig_filter_bank_measurements/cables.mat'))
            self.postfilters.append(yfcm.YigCharacterization(path%("post", i), './yig_filter_bank_measurements/cables.mat'))
            #self.prefilters.append(yfcm.YigCharacterization(f'filter_models/pre_filter_{i}.mat', './yig_filter_bank_measurements/cables.mat'))
            #self.postfilters.append(yfcm.YigCharacterization(f'filter_models/post_filter_{i}.mat', './yig_filter_bank_measurements/cables.mat'))
    
    def getFilterResponses(self, idx, f):
        i=self.prefilters[idx].computeTuningCurrent(f)
        return self.prefilters[idx].getFilterResponse(i), self.postfilters[idx].getFilterResponse(i)
    
    def getTunedS21(self, f):
        if f <= 1e9:
            return self.getFilterResponses(0,f+f/40-10e6)
        if f <= 2e9:
            return self.getFilterResponses(1,f+20e6)
        if f <= 4e9:
            if f < 3150e6:
                return self.getFilterResponses(2,f)
            else:
                return self.getFilterResponses(2,f+10e6)
        if f <= 8e9:
            return self.getFilterResponses(3,f)
        if f <= 12e9:
            return self.getFilterResponses(4,f)
        if f <= 20e9:
            return self.getFilterResponses(5,f-10e6)
        
