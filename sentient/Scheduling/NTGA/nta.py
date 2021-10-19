import itertools
import logging
import random
import sys
from typing import List
from functools import cached_property
import pickle
import os
import subprocess

import numpy as np
import shortuuid
from itertools import islice
import re

from config import save_path, strat_path, VERIFYTA

from sentient.Scheduling.NTGA import network, controlloop
from sentient import pyuppaal
import sentient.Abstractions as abstr


class nta:

    def __init__(self, net: network, control_loops: List[controlloop], synchronization=None):

        # TODO: Check compatibility w.r.t. synchronization signals
        self.net = net
        self.control_loops = control_loops
        self.ns = len(control_loops)
        self.synchronization = synchronization or ["up", "down", "timeout", "ack", "nack"]
        # self.max_early_triggers = max_early_triggers
        # Calculate common scaling factor to make UPPAAL happy
        import math
        scale = 1
        l = [a.scale for a in self.control_loops]
        for i in l:
            scale = scale * i // math.gcd(scale, i)
        self.common_scale = scale
        for s in self.control_loops:
            s.scale = scale
        logging.info(f'Set common scale to: {self.common_scale}')

    def _generate_declarations(self):
        if len(self.synchronization) == 0:
            return ""
        else:
            # return f'broadcast chan {", ".join(self.actions)};\n'
            decl = f'chan {", ".join(self.synchronization)};\n'
            decl += f'int EarNum;\n'#const int EarMax = {self.max_early_triggers};\n'
            return decl

    def _generate_system(self):
        system_ass = ''
        systems = []
        for cl in self.control_loops:
            system_ass += f'{cl.name}{cl.index} = {cl.name}();\n'
            systems.append(f'{cl.name}{cl.index}')

        system_ass += f'{self.net.name}{self.net.index} = {self.net.name}();\n'
        systems += [f'{self.net.name}{self.net.index}']
        return system_ass + f"system {', '.join(systems)};"

    def _generate_template(self):
        templates = [cl.template for cl in self.control_loops]
        templates += [self.net.template]
        return pyuppaal.NTA(templates=templates,
                            declaration=self._generate_declarations(),
                            system=self._generate_system())

    @cached_property
    def template(self):
        return self._generate_template()

    def to_xml(self):
        return self.template.to_xml()

    def export(self, file_name: str = None, export_type: str = 'txt'):
        """
        Exports the timed automaton to a specified file_type. Default is plain text
        """
        export_type = export_type.lower()
        if file_name is None:
            file_name = self.__class__.__name__
        elif export_type in ['uppaal', 'xml']:
            res = self.to_xml()
            if not file_name.endswith('.xml'):
                file_name += '.xml'
            with open(os.path.join(save_path, file_name), 'w') as f:
                f.write(res)
        elif export_type in ['pickle', 'bytes', 'byte_stream']:
            self._export_pickle(file_name)

    def _export_pickle(self, file_name: str):
        if not file_name.endswith('.pickle'):
            file_name += '.pickle'

        with open(os.path.join(save_path, file_name), 'wb') as f:
            pickle.dump(self, f, pickle.HIGHEST_PROTOCOL)

    def generate_strategy(self, parse_strategy=True, delete_files=True):

        # Throw warning if more than 2 control loops
        if len(self.control_loops) > 2:
            import warnings
            logging.warning('Using more than two control loops might take a lot of time and/or memory(!).  ')
            warnings.warn('Using more than two control loops might take a lot of time and/or memory(!).  ')

        idx = shortuuid.uuid()[:4]
        print("Exporting NTA to UPPAAL...")
        # First export NTA to xml format
        self.export(f'NTA_{idx}', 'xml')

        strat_name = f'Strategy_{idx}'
        spath = os.path.join(strat_path, strat_name) + '.q'
        with open(spath, 'w') as f:
            f.write(f'strategy {strat_name} = control: A[] not ({self.net.name}{self.net.index}.Bad)')

        temp = os.path.join(save_path, f'NTA_{idx}.xml')
        arg_list = [VERIFYTA, '-u', '-s', '--generate-strategy', '2', '--print-strategies',
                    strat_path, temp, f'{spath}']

        # arg_list = [VERIFYTA, '-h']

        # import resource
        # def limit_mem():
        #     resource.setrlimit(resource.RLIMIT_AS, (4*1024**3, resource.RLIM_INFINITY))

        print('Generating Strategy using UPPAAL...')
        verify_ta = subprocess.Popen(arg_list, stdout=subprocess.PIPE)#, preexec_fn=limit_mem)
        try:
            # Run the query in UPPAAL to generate the strategy file
            verify_ta.wait()
            # result = verify_ta.stdout.decode('utf-8')
        except KeyboardInterrupt:
            verify_ta.terminate()
            sys.exit()
        except:
            print(verify_ta.communicate())
            sys.exit()

        logging.info(f'Strategy {idx} generated. Saved in {strat_path}')

        if delete_files:
            os.remove(os.path.join(save_path, f'NTA_{idx}.xml'))
            os.remove(spath)

        if parse_strategy:

            print("Parsing Strategy...")
            # This regex string represents all states from which
            # an ack! signal is sent, i.e. when triggering must occur
            # Groups: (Region, CL1), (Region, CL2), ... , (clock conditions), (CL to trigger)
            REGEX_STRING = r'\nState: \( .*\).*'
            for cl in self.control_loops:
                REGEX_STRING += rf'{cl.name}{cl.index}+\.from_region=([0-9]+).*'

            REGEX_STRING += r'\n.*[\n]*When you are in \((.*)\).*[\n]+([A-z0-9]+)\.Trans.*, from_region := to_region.*}\n'

            with open(os.path.join(strat_path, strat_name), 'r') as strat:
                raw_strat = strat.read()
                raw_condition_tuples = re.findall(REGEX_STRING, raw_strat)

            limit_dict = {}

            for condition in raw_condition_tuples:
                self._populate_limit_dict_from_condition_tuple(condition, limit_dict)

            import json
            with open(os.path.join(strat_path, strat_name + '.json'), 'w') as file:
                json.dump(limit_dict, file)

            print(f"Saved parsed strategy to: {os.path.join(strat_path, strat_name + '.json')}")

            # if delete_files:
            #     os.remove(os.path.join(strat_path, strat_name))


    def _get_control_loops(self, lines):

        # Regex for finding CL names
        cl_regex = (".* ([A-z0-9]+)\.c==([A-z0-9]+)\.c.*")

        for line in lines:
            match_obj = re.match(cl_regex, line)
            if match_obj:
                return match_obj.groups()

        return []

    def _populate_limit_dict_from_condition_tuple(self, condition_tuple, limit_dict):

        # cl_regions = tuple(int(condition_tuple[i]) for i in range(0, len(self.closed_loops)))
        cl_regions = tuple(int(x.loc_dict_inv[int(condition_tuple[i])]) for (i,x) in enumerate(self.control_loops))

        cl_names = [f'{cl.name}{cl.index}' for cl in self.control_loops]
        if cl_regions not in limit_dict:
            limit_dict[str(cl_regions)] = []

        to_trigger = cl_names.index(condition_tuple[-1])

        # main part to decipher
        invariant_string = condition_tuple[-2]

        # there could be multiple combinations for the same region
        invariants = invariant_string.replace('(', '').replace(')', '').split('||')

        # each invariant deals with precisely one upper and lower limit per CL
        for invariant in invariants:
            # handle cases one by one
            clock_conditions = invariant.split('&&')
            clock_conditions = [condition.strip()
                                for condition in clock_conditions]

            # Store clock conditions in the form:
            # A_eq @ c == b_eq
            # A_leq @ c <= b_leq
            # Entry becomes: [(cl1l, cl1h), .., A_eq, b_eq, A_leq, b_leq, to_trigger]
            A_eq = []
            b_eq = []
            A_leq = []
            b_leq = []

            for condition in clock_conditions:
                # 1. Handle case when difference between clocks
                if '-' in condition:
                    # the case in which loop1.c - loop2.c == x needn't be handled.
                    # It is covered in other cases (where loop clocks have exact
                    # valuation matches, e.g. loop1.c == 9)
                    if '==' in condition:
                        continue

                    reg_exp_first_loop = '([A-z0-9]+)\.c-([A-z0-9]+)\.c.*[<|<=]([+-]?[0-9]+).*'
                    match_obj = re.findall(reg_exp_first_loop, condition)
                    cl1 = cl_names.index(match_obj[0][0])
                    cl2 = cl_names.index(match_obj[0][1])

                    # Fill matrices
                    a_leq_new = [0 for i in range(0, self.ns)]
                    a_leq_new[cl1] = 1
                    a_leq_new[cl2] = -1
                    A_leq.append(a_leq_new)# = np.append(A_leq, a_leq_new)
                    b_leq.append([int(match_obj[0][2])/self.common_scale])# = np.append(b_leq,np.array([int(match_obj[0][2])]))
                    continue

                # 2. Handle cases in which clock are equal
                eq_regex = '([A-z0-9]+)\.c(<|<=|==)([A-z0-9]+)\.c'
                match_obj = re.findall(eq_regex, condition)
                if match_obj != []:
                    cl1 = cl_names.index(match_obj[0][0])
                    cl2 = cl_names.index(match_obj[0][2])

                    if match_obj[0][1] == '==':
                        # Fill matrices
                        a_eq_new = [0 for i in range(0, self.ns)]
                        a_eq_new[cl1] = 1
                        a_eq_new[cl2] = -1
                        A_eq.append(a_eq_new)# = np.append(A_eq, a_eq_new)
                        b_eq.append([0])# = np.append(b_eq,[np.array([0])])
                    elif match_obj[0][1] == '<=':
                        a_leq_new = [0 for i in range(0, self.ns)]
                        a_leq_new[cl1] = 1
                        a_leq_new[cl2] = -1
                        A_leq.append(a_leq_new)  # = np.append(A_eq, a_eq_new)
                        b_leq.append([0])  # = np.append(b_eq,[np.array([0])])

                    continue

                # 3. Handle cases from the individual clock limits
                for cl, name, reg in zip(self.control_loops, cl_names, cl_regions):
                    if name in condition:
                        # condition = condition.replace(name, '')
                        lims = self._extract_limits(condition, cl.invariants[f'R{reg}'][1] - cl.max_early, cl.invariants[f'R{reg}'][1] + cl.max_delay_steps)
                        if lims is None or lims[0] is None or lims[1] is None:
                            print(f'None with condition: {condition}')
                        elif lims[0] == lims[1]:
                            # Add to eq
                            a_eq_new = [0 for i in range(0, self.ns)]
                            idx = cl_names.index(name)
                            a_eq_new[idx] = 1
                            A_eq.append(a_eq_new)# = np.append(A_eq, a_eq_new)
                            b_eq.append([lims[0]/self.common_scale])# = np.append(b_eq, np.array([lims[0]]))
                        else:
                            a_leq_new1 = [0 for i in range(0, self.ns)]
                            a_leq_new2 = [0 for i in range(0, self.ns)]
                            idx = cl_names.index(name)
                            a_leq_new1[idx] = -1
                            a_leq_new2[idx] = 1
                            A_leq.append(a_leq_new1)# = np.append(A_leq, a_leq_new1)
                            A_leq.append(a_leq_new2)# = np.append(A_leq, a_leq_new2)
                            b_leq.append([-lims[0]/self.common_scale])# = np.append(b_leq, np.array([-lims[0]]))
                            b_leq.append([lims[1]/self.common_scale])# = np.append(b_leq, np.array([lims[1]]))

            expr = tuple((A_eq, b_eq, A_leq, b_leq, to_trigger))
            limit_dict[str(cl_regions)].append(expr)

    def _extract_limits(self, condition, cl_low, cl_high):

        # NOTE: UPPAAL doesn't have operators of the kind "> or >=", but only "< or <="

        lower_lim = cl_low
        upper_lim = cl_high

        # First define the regex strings
        regex_string_lower_limit = '.*([0-9]+)([<>]=?|==).*'
        regex_string_upper_limit = '.*([<>]=?|==)([0-9]+).*'

        # Check for upper limit
        match_obj = re.match(regex_string_upper_limit, condition)
        if match_obj:
            oper = match_obj.group(1)
            val = int(match_obj.group(2))

            if oper == "==":
                return (val, val)
            elif oper == "<=":
                upper_lim = val
            elif oper == "<":
                upper_lim = val - 1

            return (lower_lim, upper_lim)

        # Check for lower limit
        match_obj = re.match(regex_string_lower_limit, condition)
        if match_obj:
            val = int(match_obj.group(1))
            oper = match_obj.group(2)

            if oper == "==":
                return (val, val)
            elif oper == "<=":
                lower_lim = val
            elif oper == "<":
                lower_lim = val + 1

            return (lower_lim, upper_lim)

