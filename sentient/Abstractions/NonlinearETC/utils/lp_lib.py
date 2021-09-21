import numpy as np
from scipy.optimize import linprog
import logging

class LPdata(object):
    """Stores and manipulates the Linear programming data
    
    Constructor arguments:
        s (object of class Spec): Specification of the Spec class
    optional arguments:
        C (numpy 1-d matrix): LP cost function vector c, s.t. the cost function is c^T.x.  
            Dimension: p + 1 (approximation order plus 1).
        D (tuple): bounds on x. Dimension: p + 1 (approximation order plus 1)
        
    Attributes:
        Spec: specification
        A: A matrix
        B: B matrix
        C: objective vector
        D: bounds on the optimiztion variables
        solutions: Found solutions of the LP iterations
    """
    def __init__ (self, s,C =[],D =[]):
        """Constructor"""
        self.initialize_LP(s,C,D)
        self.solutions = [];
    
    def initialize_LP(self,s,C_u =[],D_u=[]):
        """Create the data for the LP sovler"""
        self.A=[]
        self.B=[]

        #Create A and B matrix
        for point in s.Samples: #given the sample points from Spec
            self.append_constraint(s,point,0)
        
        if C_u !=[]: #if cost function is given
            if len(C_u) == s.p+1:
                self.C = C_u
            else: 
                logging.info('User defined c vector is of the wrong dimension. Default cost function used instead')
        else: #if cost functionis not given use default
           self.C = np.zeros(s.p+1)
           self.C[s.p-1]=0
           self.C[s.p]=1 
           
        if D_u !=[]: #if bounds are given
            if len(D_u) == s.p+1:
                self.D = D_u
            else: 
                logging.info('User defined bounds D are of the wrong dimension. Default bounds used instead')
        else: #if bounds are not given, use default bounds
            self.D = []
            for d in range(1,s.p):
                self.D.append((None,None))
            self.D.append((0,None)) #Delta_n
            self.D.append((0,None)) #Gamma >= 0
            self.D = tuple(self.D)
            
        return [self.A,self.B,self.C,self.D]

    def LP_solve(self,lp_method = 'revised simplex'):
        """Solves the LP problem"""
        logging.info("Initialising Optimization")
        Verbose=False
        if (lp_method == 'interior-point'):
            res = linprog(self.C, A_ub=self.A, b_ub=self.B, bounds=self.D ,options=dict(disp=Verbose, tol=1e-12, presolve=True, lstsq=True),method = lp_method)
        else:
            res = linprog(self.C, A_ub=self.A, b_ub=self.B, bounds=self.D ,options=dict(disp=Verbose, tol=1e-12),method = lp_method)
        try:
            self.solutions.append(list(res.x))
            return 1
        except TypeError:
            logging.error("LP optimization failed, terminating script")
            return -1
        
      
    def append_constraint(self,s,point,slack):
        """INPUTS:
            s (object of class Spec)
            point (list): the counterexample point that is to be added as a constraint to the LP
            slack (float): the slack variable
        """
        lies = list(s.Lie)
        lie_computed = []
        all_vars = list(s.Symbolic_variables)
        for i in range(0,s.p): #evaluate all lie derivatives at the given point
            lie_computed.append(-float(lies[i].subs(zip(all_vars, point))))
        An0 = lie_computed[0:s.p-1]
        An1 =An0 +[-1.0,0.0]
        An2 = list(-np.array(An0))+[1.0, -1.0]
        Bn1 = lie_computed[s.p-1]
        Bn2 = -Bn1
        #positivity constraint d_0*fi(x=x_0) + d_p > 0
        dic = {}
        for i in range(0,int(s.n/2)):
            dic[s.State[i]] = s.Init_cond_symbols[i]
        fi_init_cond = lies[0].subs(dic)    
        fi_init_cond_computed = float(fi_init_cond.subs(zip(all_vars, point)))
        An3 = list(np.zeros(s.p+1,))
        An3[0] = -fi_init_cond_computed
        An3[-2] = -1
        Bn3 = 0
        
        self.A.append(An1)
        self.A.append(An2)
        self.B.append(Bn1-2*slack)
        #self.B.append(Bn1-1.1*slack)
        self.B.append(Bn2)
        #if s.p>2:
        self.A.append(An3) #for positivity
        self.B.append(Bn3) #for positivity
        
#    def cut_out_deltas(self,epsilon=0.001):
#        "IN DEVELOPMENT. Carve out a piece of the search space"
#        Ax =np.matmul(np.array(self.A),np.array(self.solutions[-1]))
#        B = np.array(self.B)
#        C = Ax-B
#        #Arrays close to zero
#        inds =np.isclose(C,np.zeros(len(C)))
#        A0 = (np.array(self.A)[inds])
#        # make the constraint stronger
#        B0 = (np.array(self.B)[inds])
#        #Average A matrix
#        An = np.mean(A0,axis= 0).tolist()
#        Bn = (np.mean(B0)-epsilon).tolist()
#        self.A.append(An)
#        self.B.append(Bn)