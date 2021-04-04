class AbstractKFilter:

    def reset(self):
        self.steady_state = False
        self.state = np.zeros(self.s)
        self.P = np.zeros((self.s, self.s))


class KFilter(AbstractKFilter):
    def __init__(self, A, B, Q, C, R, state=None):
        self.A = A
        self.Q = Q
        self.C = C
        self.R = R
        self.B = B
        self.s = A.shape[0]
        self.m = C.shape[0]
        if state is None:
            self.state = np.zeros(self.s)
        else:
            self.state = state
        self.prev_P = np.zeros((self.s, self.s))
        self.P = np.zeros((self.s, self.s))
        self.steady_state = False
    
    def predict(self, control):
        self.prev_P = copy.deepcopy(self.P)
        self.state = self.A@self.state + self.B@control
        self.P = self.A@self.prev_P@self.A.T + self.Q
        
    def update(self, measurement):
        if not self.steady_state:
            self.K = self.P@self.C.T@np.linalg.inv(self.C@self.P@self.C.T + self.R)
            self.P = (np.identity(self.s) - self.K@self.C)@self.P
            if np.allclose(self.P, self.prev_P):
                self.steady_state = True
        innovation = measurement - self.C@self.state
        self.state = self.state + self.K@innovation
        
