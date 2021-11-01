import sys

import sympy
import numpy as np
import collections
import os, os.path
import subprocess
import logging

from config import root_path

sym_dict = collections.OrderedDict({"Add":"+","Mul":"*","Pow":"^",
           "StrictGreaterThan":">","GreaterThan":">=",
           "StrictLessThan":"<","LessThan":"<=",
           "And":"and", "Or":"or","Not":"not", "exp":"exp", "sin":"sin", "cos":"cos", "Abs":"abs"})

num_dict =["Float","Integer","Zero","One","Symbol","NegativeOne"]


def sympy2matlab(symbolic_expression):
    string = str(symbolic_expression)
    string = string.replace("**","^")
    string = string.replace("Abs","abs")
    return string

def set2str(symbolic_expression):
    text = "(and "
    for expr in symbolic_expression.args:
        text = text + "(" + sympy2matlab(expr) + ") "
    text = text + ")"
    return text    
    
def write_drh_file(init_symbolic, goal_symbolic, time_max, symbol_list, dynamics, file_path,file_name="file.drh"):
    """ Create a DRH file for reachability analysis."""
    
    text = ""
    #Check file suffix
    if not file_name.endswith('.drh'):
        file_name = file_name + '.drh'
    #Define box of operation    
    for var in symbol_list:
        text = text + "[-1000,1000] " + str(var) + ";\n"
    text = text + "[-10,10] tau;\n"    
    text = text + "[0," + str(time_max) + "] time;\n\n" 
    text = text + "{ mode 1;\n"
    text = text + "invt:\n"
    text = text + "(" + str(var) + " <= 1000);\n"
    text = text + "flow:\n"
    i = 0
    for var in symbol_list:
        text = text + "d/dt[" + str(var) + "] = " + sympy2matlab(dynamics[i]) + ";\n"
        i = i+1
    text = text + "d/dt[tau] = 1;\n"
    text = text + "jump:\n (and (tau=0)) ==> @1 (and "
    for var in symbol_list:
        text = text + "(" + str(var) + "' = " + str(var) + ") "
    text = text + "(tau' = tau));\n"
    text = text + "}\n\n"
    text = text + "init:\n" + "@1 " + set2str(init_symbolic) + ";\n\n"  
    text = text + "goal:\n" + "@1 " + set2str(goal_symbolic) + ";\n\n"
    
    with open(os.path.join(file_path,file_name),'w+') as f:
        f.write(text)

    logging.info("DRH File exported at " + os.path.join(file_path,file_name))
    return 
        
def call_dReach(dreach_path,file_path,file_name ="file.drh",dreal_precision = 0.01, time_out = None):
    # Check file suffix
    if not file_name.endswith('.drh'):
        file_name = file_name + '.drh'
    #Initialize results
    result ={}
    logging.info("Calling dReach")
    try:

        word = dreach_path + ' -k 0' + ' -l 0 ' + os.path.relpath(file_path) + '/' + \
               file_name + ' --precision ' + str(dreal_precision) + \
               ' --polytope'

        if (time_out == None):
            outputdReach = subprocess.check_output([word],shell=True).decode("utf-8")
        else:
            outputdReach = subprocess.check_output([word],shell=True, timeout=time_out).decode("utf-8")

    except KeyboardInterrupt:
        # Make sure the processes are killed when keyboardinterrupt
        subprocess.run(["pkill", "-f", "dReal"])
        subprocess.run(["pkill","-f","dReal"])
        subprocess.run(["pkill","-f","dReal"])
        subprocess.run(["pkill","-f","dReal"])

        sys.exit()

    except Exception:
        # logging.info(outputdReach.communicate())
        outputdReach = 'time-out'
        result['time-out']= True
        logging.info("dReach time-out or other unexpected result.")

    #Process data: unsat = 1, delta-sat = 0 (unsat proofs the inequality)
    if outputdReach == 'time-out':
        result['sat'] = True
        result['time-out']= True
    elif ('all of them are unsat' in outputdReach):
        result['sat']=False
        result['time-out'] = False
    else:
        result['sat'] = True
        result['time-out']= False
    return result    

def dReach_verify(init_symbolic, goal_symbolic, time_max, symbol_list, dynamics, dreal_precision,dreach_path,file_path,file_name ="file.drh", time_out = None):
    # Check file suffix
    if not file_name.endswith('.drh'):
        file_name = file_name + '.drh'
    
    write_drh_file(init_symbolic, goal_symbolic, time_max, symbol_list, dynamics, file_path,file_name)
    
    result = call_dReach(dreach_path,file_path,file_name,dreal_precision, time_out)
    for fname in os.listdir(file_path):
        if fname.startswith(os.path.splitext(file_name)):
            os.remove(os.path.join(file_path, fname))

    return result
    
    
    
    
    
    