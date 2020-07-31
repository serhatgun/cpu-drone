import math
import time

class pid(object):

    def __init__(self, P = 0, I = 0, D = 0, maxI = 0):
        self.pGain = P
        self.iGain = I
        self.dGain = D
        self.iMax = abs(maxI)
        self.integrator = 0
        self.lastError = None
        self.lastUpdate = time.time()

    #Returns time difference since last get_dt call
    def get_dt(self, max_dt):
        now = time.time()
        time_diff = now - self.lastUpdate
        self.lastUpdate = now
        if time_diff > max_dt:
            return 0.0
        else:
            return time_diff

    #Return P term
    def getP(self, error):
        return self.pGain * error

    #Return I term
    def getI(self, error, dt):
        self.integrator = self.integrator + error * self.iGain * dt
        self.integrator = min(self.integrator, self.iMax)
        self.integrator = max(self.integrator, -self.iMax)
        return self.integrator

    #Return D term
    def getD(self, error, dt):
        if self.lastError is None:
            self.last_error = error
        ret = (error - self.last_error) * self.dGain * dt
        self.last_error = error
        return ret

    #Return P + I terms
    def getPI(self, error, dt):
        return self.getP(error) + self.getI(error,dt)

    #Return P + I + D terms
    def getPID(self, error, dt):
        return self.getP(error) + self.getI(error,dt) + self.getD(error, dt)

    # Return built up I term
    def getIntegrator(self):
        return self.integrator

    #Clears I term
    def resetIntegrator(self):
        self.integrator = 0