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
                json.dump({str(k): v for (k,v) in limit_dict.items()}, file)

            print(f"Saved parsed strategy to: {os.path.join(strat_path, strat_name + '.json')}")
            self.scheduler = limit_dict
            if delete_files:
                os.remove(os.path.join(strat_path, strat_name))


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
            limit_dict[cl_regions] = []

        to_trigger = cl_names.index(condition_tuple[-1])

        # main part to decipher
        invariant_string = condition_tuple[-2]

        # there could be multiple combinations for the same region
        invariants = invariant_string.replace('(', '').replace(')', '').split('||')

        A_eq = []
        b_eq = []
        A_leq = []
        b_leq = []

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
                        # Check for lower limit
                        regex_string_lower_limit = '([0-9]+)([<>]=?|==).*'
                        match_obj = re.match(regex_string_lower_limit, condition)
                        if match_obj:
                            val = int(match_obj.group(1)) / self.common_scale
                            oper = match_obj.group(2)
                            if oper == "==":
                                a_eq_new = [0 for i in range(0, self.ns)]
                                idx = cl_names.index(name)
                                a_eq_new[idx] = 1
                                A_eq.append(a_eq_new)# = np.append(A_eq, a_eq_new)
                                b_eq.append([val])# = np.append(b_eq, np.array([lims[0]]))
                            else:
                                if oper == "<=":
                                    lower_lim = val
                                elif oper == "<":
                                    lower_lim = val - 0.00001

                                a_leq_new = [0 for i in range(0, self.ns)]
                                idx = cl_names.index(name)
                                a_leq_new[idx] = -1
                                A_leq.append(a_leq_new)  # = np.append(A_leq, a_leq_new1)
                                b_leq.append([-lower_lim])  # = np.append(b_leq, np.array([-lims[0]]))

                        # Check for upper limit:
                        regex_string_upper_limit = '.*([<>]=?|==)([0-9]+)'
                        match_obj = re.match(regex_string_upper_limit, condition)
                        if match_obj:
                            oper = match_obj.group(1)
                            val = int(match_obj.group(2)) / self.common_scale
                            if oper == "==":
                                a_eq_new = [0 for i in range(0, self.ns)]
                                idx = cl_names.index(name)
                                a_eq_new[idx] = 1
                                A_eq.append(a_eq_new)  # = np.append(A_eq, a_eq_new)
                                b_eq.append([val])  # = np.append(b_eq, np.array([lims[0]]))
                            else:
                                if oper == "<=":
                                    upper_lim = val
                                elif oper == "<":
                                    upper_lim = val - 0.00001

                                a_leq_new = [0 for i in range(0, self.ns)]
                                idx = cl_names.index(name)
                                a_leq_new[idx] = 1
                                A_leq.append(a_leq_new)  # = np.append(A_leq, a_leq_new1)
                                b_leq.append([upper_lim])  # = np.append(b_leq, np.array([-lims[0]]))

            expr = tuple((A_eq, b_eq, A_leq, b_leq, to_trigger))
            limit_dict[cl_regions].append(expr)

    def _extract_limits(self, condition, cl_low, cl_high):

        # NOTE: UPPAAL doesn't have operators of the kind "> or >=", but only "< or <="

        lower_lim = max(0,cl_low)
        upper_lim = cl_high

        # First define the regex strings
        regex_string_lower_limit = '.*([0-9]+)([<>]=?|==).*'
        regex_string_upper_limit = '.*([<>]=?|==)([0-9]+).*'

        # Check for upper limit
        match_obj = re.match(regex_string_upper_limit, condition)
        if match_obj:
            oper = match_obj.group(1)
            val = int(match_obj.group(2))/self.common_scale

            if oper == "==":
                return (val, val)
            elif oper == "<=":
                upper_lim = val
            elif oper == "<":
                upper_lim = val - 0.001

            return (lower_lim, upper_lim)

        # Check for lower limit
        match_obj = re.match(regex_string_lower_limit, condition)
        if match_obj:
            val = int(match_obj.group(1))/self.common_scale
            oper = match_obj.group(2)

            if oper == "==":
                return (val, val)
            elif oper == "<=":
                lower_lim = val
            elif oper == "<":
                lower_lim = val + 0.001

            return (lower_lim, upper_lim)

    def simulate(self, Ts: float = 0.01, Tmax: float = 1, x0=None, use_scheduler=True):

        if any([type(cl) == abstr.TrafficModelNonlinearETC for cl in self.control_loops]):
            raise NotImplementedError

        # Check correct/enough initial conditions
        if x0 is None:
            x0 = [np.random.uniform(low=-4, high=4, size=(cl.abstraction.plant.nx,)) for cl in self.control_loops]
        else:
            if len(x0) != len(self.control_loops):
                print('Supply initial conditions for each control loop.')
                return

            for x0i, cl in zip(x0, self.control_loops):
                if len(x0i) != cl.abstraction.plant.nx:
                    print(
                        f'Initial condition dimension ({len(x0i)}) does not correspond to the expected ({cl.abstraction.plant.nx}).')
                    return

            x0 = [np.array(x) for x in x0]

        # 3D Matrix storing the evolution of the continuous states over time.
        x = [[np.array(x0i)] for x0i in x0]
        xhat = [[np.array(x0i)] for x0i in x0]
        u_hist = [[] for i in range(0, self.ns)]  # continuous inputs


        # Evolution of the traffic model regions over time
        regions = [[cl.abstraction.region_of_state(x0i)] for (x0i, cl) in zip(x0, self.control_loops)]

        s = [[str(cl.loc_dict['_'.join([str(i) for i in loc[0]])])] for (loc, cl) in zip(regions, self.control_loops)]
        ss = tuple(q[-1] for q in s)
        print(ss)
        clocks = [[0] for i in range(0, self.ns)]

        for i in range(0, self.ns):
            print(f'Controlloop {i} starts in region {regions[i][0]}')

        TriggerTimes = [[0] for i in range(0, self.ns)]
        CollisionTimes = {}

        N = int(Tmax / Ts)  # Number of samples

        import scipy
        I = [scipy.integrate.quad_vec(lambda s: scipy.linalg.expm(cl.abstraction.plant.A * s), 0, Ts)[0] for cl in
             self.control_loops]


        for t in range(0, N):
            # Step 1: Update the continuous states
            utemp = [cl.abstraction.controller.K @ xn[-1] for (cl, xn) in zip(self.control_loops, xhat)]
            xn = [scipy.linalg.expm(cl.abstraction.plant.A * Ts) @ xi[-1] + integral @ cl.abstraction.plant.B @ ui
                  for (cl, xi, ui, integral) in zip(self.control_loops, x, utemp, I)]

            for i in range(0, self.ns):
                x[i].append(xn[i])
                xhat[i].append(xhat[i][-1])
                u_hist[i].append(utemp[i])
                clocks[i].append(clocks[i][-1] + Ts)


            ## Step 2: Check triggering conditions
            # If a scheduler is defined use that
            if self.scheduler is not None and use_scheduler:
                to_trigger = -1
                ss = tuple(q[-1] for q in s)
                if ss not in self.scheduler:
                    to_trigger = random.randint(0, self.ns-1)
                    print(f'State {ss} not in scheduler => Choose to trigger {to_trigger+1}')
                else:
                    cc = np.array([[c[-1]] for c in clocks])
                    print(f'Current Clock values: {cc}')
                    for (Aeq, beq, Aleq, bleq, trig) in self.scheduler[ss]:
                        if Aeq != [] and any([i != 0 for i in (Aeq @ cc - beq)]):
                            continue
                        if Aleq != [] and any([i > 0 for i in Aleq @ cc - bleq]):
                            continue

                        to_trigger = trig
                        print(f'Loop {trig+1} should trigger')
                        break

                for i in range(0, self.ns):
                    if to_trigger == i:
                        reg = self.control_loops[i].abstraction.region_of_state(x[i][-1])
                        si = str(cl.loc_dict['_'.join([str(i) for i in reg])])
                        clocks[i][-1] = 0
                        s[i].append(si)
                        xhat[i][-1] = xn[i]
                        regions[i].append(reg)
                        TriggerTimes[i].append(t * Ts)

                    else:
                        # reg = self.control_loops[i].abstraction.region_of_state(x[i][-1])
                        regions[i].append(regions[i][-1])
                        s[i].append(s[i][-1])

            else:
                triggers = set()
                for i in range(0, self.ns):

                    triggered = False
                    if type(self.control_loops[i].abstraction) == abstr.TrafficModelLinearPETC:
                        xx = np.block([x[i][-1], xhat[i][-1]])
                        triggered = xx.T @ self.control_loops[i].abstraction.trigger.Qbar @ xx.T > 0 or \
                                    (t * Ts - TriggerTimes[i][-1]) >= self.control_loops[i].tau_max
                    else:
                        xe = x[i][-1] - xhat[i][-1]
                        xdict = {i:j for (i,j) in zip([x, xe], self.control_loops[i].abstraction.original_state)}
                        print(xdict)
                        sys.exit()

                    if triggered:
                        xhat[i][-1] = xn[i]
                        TriggerTimes[i].append(t * Ts)
                        triggers.add(i)

                    reg = self.control_loops[i].abstraction.region_of_state(x[i][-1])
                    regions[i].append(reg)


                if len(triggers) > 1:
                    CollisionTimes[t * Ts] = triggers

        import matplotlib.pyplot as plt

        dur = np.arange(0, Ts * N, Ts)
        for i in range(0, self.ns):
            plt.plot(dur, x[i][0:len(dur)], '--')
            plt.gca().set_prop_cycle(None)
            plt.plot(dur, xhat[i][0:len(dur)])
            plt.title(f'Controlloop {i + 1}: $x(t)$ and $x_e(t)$.')
            plt.show()

        for i in range(0, self.ns):
            plt.plot(dur, u_hist[i][0:len(dur)])
            plt.title(f'Controlloop {i + 1}: $u(t)$.')
            plt.show()

        for i in range(0, self.ns):
            plt.plot(TriggerTimes[i], i * np.ones(len(TriggerTimes[i])), 'x')

        for t, ii in CollisionTimes.items():
            for i in ii:
                plt.plot(t, i, 'dk')

        plt.title('Trigger times')
        plt.yticks(range(0, self.ns), [f'Controlloop {i}' for i in range(1, self.ns + 1)])
        plt.show()

        for i in range(0, self.ns):
            plt.plot(dur, regions[i][0:len(dur)])

        plt.title('Traffic Model Regions')
        plt.legend([f'Controlloop {i}' for i in range(1, self.ns + 1)], loc='upper left')
        plt.show()