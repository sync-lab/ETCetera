
import collections
import os, os.path
import subprocess
import logging
import sys

sym_dict = collections.OrderedDict({"Add":"+","Mul":"*","Pow":"^",
           "StrictGreaterThan":">","GreaterThan":">=",
           "StrictLessThan":"<","LessThan":"<=",
           "And":"and", "Or":"or","Not":"not", "exp":"exp", "sin":"sin", "cos":"cos", "Abs":"abs"})

num_dict =["Float","Integer","Zero","One","Symbol","NegativeOne"]


def sympy2matlab(symbolic_expression):
    string = str(symbolic_expression)
    string = string.replace("**","^")
    return string

def goal_set2str(symbolic_expression):
    text = ""
    for expr in symbolic_expression.args:
        text = text + sympy2matlab(expr) + "\n"
    return text    
    
def write_model_file(init_box, goal_symbolic, time_max, symbol_list, dynamics, parameter_list, parameter_box, remainder, file_path,file_name="file.model"):
    """ Create a model file for reachability analysis."""
    
    #Check file suffix
    if not file_name.endswith('.model'):
        file_name = file_name + '.model'
    #Fix parameters of flowstar
    text = "continuous reachability\n" + "{\n" + "state var "
    for var in symbol_list:
        text = text + str(var) + ", "
    text = text + "tau\n\n" + "setting\n" + "{\n" + "adaptive steps { min 0.000000001, max 0.1}\n" +\
    "time " + str(time_max) + "\nremainder estimation " + str(remainder) + "\nidentity precondition\n" + "gnuplot octagon " +\
    str(symbol_list[0]) + ", " + str(symbol_list[1]) + "\n" + "fixed orders 4\n" + "cutoff 1e-20\n" +\
    "precision 256\n" + "output reach\n" + "print off\n}\n\n"
    
    #Define dynamics of the system
    text = text + "poly ode 2\n{\n"
    i=0
    for var in symbol_list:
        text = text + str(var) + "' = " + sympy2matlab(dynamics[i]) + "\n"
        i = i+1
    text = text + "tau' = 1\n}\n\n"
    i=0    
    for par in parameter_list:
        text = text.replace(str(par), str(parameter_box[i]))
        i = i+1
    #Define initial set
    text = text + "init\n{\n"
    i=0
    for var in symbol_list:
        text = text + str(var) + " in " + str(init_box[i]) + "\n"
        i = i+1
    text = text + "tau in [0,0]\n}\n}\n\n"
    
    #Define goal set   
    text = text + "unsafe set\n{\n"
    text = text +  goal_set2str(goal_symbolic) + "}"
    
    with open(os.path.join(file_path,file_name),'w+') as f:
        f.write(text)

    logging.info("Model File exported at " + os.path.join(file_path,file_name))
    return 
        
def call_flowstar(flowstar_path,file_path,file_name ="file.model", time_out = None):
    # Check file suffix
    if not file_name.endswith('.model'):
        file_name = file_name + '.model'
    #Initialize results
    result ={}

    logging.info("Calling flowstar")
    result['time-out']= False
    try:

        word = flowstar_path + ' < ' + os.path.relpath(file_path) + '/'+file_name
        # print(word)
        if (time_out == None):
            output_flowstar = subprocess.check_output([word],shell=True).decode("utf-8")
        else:
            output_flowstar = subprocess.check_output([word],shell=True,timeout=time_out).decode("utf-8")

    except KeyboardInterrupt:
        # Make sure the processes are killed when keyboardinterrupt
        subprocess.run(["pkill", "-f", "flowstar"])
        subprocess.run(["pkill","-f","flowstar"])
        subprocess.run(["pkill","-f","flowstar"])
        subprocess.run(["pkill","-f","flowstar"])

        sys.exit()

    except Exception:
        output_flowstar = 'time-out'
        result['time-out']= True
        logging.info("flowstar time-out or other unexpected result.")
    subprocess.run(["pkill","-f","flowstar"])
    subprocess.run(["pkill","-f","flowstar"])
    subprocess.run(["pkill","-f","flowstar"])
    subprocess.run(["pkill","-f","flowstar"])
    if output_flowstar == 'time-out':
        result['sat'] = True
    if ('SAFE' in output_flowstar):
        result['sat']= False
        result['time-out'] = False
    else:
        result['sat'] = True
    if ('is not large enough' in output_flowstar):
        result['time-out']= True
        result['sat'] = True
        logging.info("flowstar remainder is not large enough. Terminate the program and enlarge the remainder.")
    return result    

def flowstar_verify(init_box, goal_symbolic, time_max, symbol_list, dynamics, parameter_list, parameter_box,\
                  flowstar_path,file_path,file_name ="file.model", time_out = None, remainder = 1e-1,):
    # Check file suffix
    if not file_name.endswith('.model'):
        file_name = file_name + '.model'
    
    write_model_file(init_box, goal_symbolic, time_max, symbol_list, dynamics, parameter_list, parameter_box, remainder, file_path,file_name)
    
    result = call_flowstar(flowstar_path,file_path,file_name, time_out)
    return result
    
    
    
    
    
    