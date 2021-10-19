# -*- coding: utf-8 -*-
import sys
import sympy
import numpy as np
import collections
import os, os.path
import subprocess
import logging

sym_dict = collections.OrderedDict({"Add": "+", "Mul": "*", "Pow": "^",
                                    "StrictGreaterThan": ">", "GreaterThan": ">=",
                                    "StrictLessThan": "<", "LessThan": "<=",
                                    "And": "and", "Or": "or", "Not": "not", "exp": "exp", "sin": "sin", "cos": "cos",
                                    "Abs": "abs"})

num_dict = ["Float", "Integer", "Zero", "One", "Symbol", "NegativeOne"]


def symname2lisp(fun):
    """Translator function between symbolic python and SMT2 function names.
    Part of the sym2lisp translator"""
    expr = fun
    for word in sym_dict:
        expr = expr.replace(word, sym_dict[word])
    return expr


def sym2lisp(expr):
    """Translator function between symbolic python expression and SMT2 expression"""
    name = expr.func.__name__
    if name in num_dict:
        sform = str(expr)
    else:
        sform = "(" + symname2lisp(name)
        for arg in expr.args:
            sform = sform + " " + sym2lisp(arg)
        sform = sform + ")"
    return sform


def writeSMT2file(symbolic_expr, domain, domain_par, symbol_list, dReal_precision, file_path, file_name="file.smt2"):
    """ Create and SMT file to check whether inequality symbolic_expr is satisfied using dReal."""

    # Check file suffix
    if not file_name.endswith('.smt2'):
        file_name = file_name + '.smt2'
    # Write settings in a string
    text = "(set-logic QF_NRA)\n"
    # text = text + "(set-info :precision "+ str(dReal_precision)+" )\n"
    # add declaration of constants
    for var in symbol_list:
        text = text + "(declare-fun " + str(var) + " () Real)\n"
    # define domain
    text = text + "(assert " + sym2lisp(domain) + ")\n"
    if (domain_par != None):
        text = text + "(assert " + sym2lisp(domain_par) + ")\n"
    # Add inequality
    text = text + "(assert (not  " + sym2lisp(symbolic_expr) + "))\n"
    # check satisfiability
    text = text + "(check-sat)\n"
    text = text + "(exit)"

    with open(os.path.join(file_path, file_name), 'w+') as f:
        f.write(text)
    logging.info("SMT2 File exported at " + os.path.join(file_path, file_name))
    return


def call_dReal(dreal_path, file_path, file_name="file.smt2", dreal_precision=0.001, t_max=None):
    """ Call dReal and return a dictionary containing whether the inequality
    is satisfied, if not, the violations stored as a string, and a time-out flag
    """
    # Check file suffix
    if not file_name.endswith('.smt2'):
        file_name = file_name + '.smt2'

    # Initialize results
    result = {}
    logging.info("Calling dReal")
    try:
        # dReal
        temp = [dreal_path, '--polytope', '--model', '--precision', str(dreal_precision),
                os.path.join(file_path, file_name)]
        if t_max == None:
            outputdReal = subprocess.check_output(temp).decode("utf-8")
        else:
            outputdReal = subprocess.check_output(temp, timeout=t_max).decode("utf-8")

    except KeyboardInterrupt:
        # Make sure the processes are killed when keyboardinterrupt
        subprocess.run(["pkill", "-f", "dReal"])
        subprocess.run(["pkill","-f","dReal"])
        subprocess.run(["pkill","-f","dReal"])
        subprocess.run(["pkill","-f","dReal"])

        sys.exit()

    except Exception:
        outputdReal = 'time-out'
        result['time-out'] = True
        logging.info("dReal time-out ({} seconds or other unexpected result.)".format(t_max))
    # Process data: unsat = 1, delta-sat = 0 (unsat proves the inequality)
    if outputdReal == 'time-out':
        result['sat'] = False
        result['violation'] = None
    elif outputdReal == 'unsat\n':
        result['sat'] = True
        result['time-out'] = False
        result['violation'] = None
        logging.info('Inequality Satisfied')
    else:
        result['sat'] = False
        result['time-out'] = False
        result['violation'] = outputdReal
        logging.info('Inequality not verified')
    return result


def readViolations(dreal_result, symbol_list):
    """ Read violations. Note that accuracy might be lost due to the use of eval()"""
    string = dreal_result
    for var in symbol_list:
        string = string.replace(str(var), '"' + str(var) + '"')
    array = string.splitlines()
    array = array[1:]
    # Store point as a dictionary
    viol_dict = {}
    ######### This is for dreal4
    for entry in array:
        d = eval('{' + entry + '}')
        viol_dict.update(d)
    ######## This is for dreal3
    #    for entry in array[:-1]:
    #        entry2=entry.replace(' [ ENTIRE ] =','')
    #        d = eval('{'+entry2 +'}')
    #        viol_dict.update(d)
    # Take the first vertex as counter example
    new_sample = ()
    for var in symbol_list:
        #        new_sample = new_sample +(viol_dict[str(var)][0],)
        new_sample = new_sample + (np.mean(viol_dict[str(var)]),)
    return new_sample


def dReal_verify(symbolic_expr, domain, domain_par, symbol_list, dReal_precision, dreal_path, file_path,
                 file_name="file.smt2", time_out=None):
    """ Main wrapper function"""
    # Check file suffix
    if not file_name.endswith('.smt2'):
        file_name = file_name + '.smt2'
    # Write SMT file
    writeSMT2file(symbolic_expr, domain, domain_par, symbol_list, dReal_precision, file_path, file_name)
    # Call dReal
    result = call_dReal(dreal_path, file_path, file_name, dreal_precision=dReal_precision, t_max=time_out)
    # Read violations
    if result['violation'] != None and result['time-out'] == False:
        result['violation'] = readViolations(result['violation'], symbol_list)
        logging.info("Violation at" + str(result['violation']))

    os.remove(os.path.join(file_path, file_name))

    return result
