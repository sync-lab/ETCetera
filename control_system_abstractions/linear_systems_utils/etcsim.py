#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Sep 19 15:02:31 2018

@author: ggleizer
"""
import numpy as np
import warnings
from scipy import integrate
import logging

import control_system_abstractions.linear_systems_utils.linearetc as etc


''' Settings for solvers '''
_MAX_TRIES_TIGHT_LYAPUNOV = 10
_ASSERT_SMALL_NUMBER = 1e-6
_LMIS_SMALL_IDENTITY_FACTOR = 1e-6

''' etcsim.py '''
''' Functionality to simulate ETC/STC and general sample-and-hold systems'''


class ETCSimWarning(Warning):
    pass


class ETCSimError(Exception):
    pass


'''
    SIMULATIONS
'''


def simulate_sample_and_hold_control(sohc: etc.SampleAndHoldController,
                                     ts, xp0, xc0=None,
                                     disturbance=None, noise=0,
                                     xs0=np.array([0,])):
    p = sohc.plant
    c = sohc.controller
    if sohc._sampling_type not in {'ETC', 'STC', 'Periodic'}:
        raise ETCSimError('input must be either ETC, STC, or periodic')

    # Check if there is a state evolution function for the scheduler
    scheduler_has_states = True
    try:
        sohc.fs(xs0)
    except AttributeError:
        scheduler_has_states = False

    xp0 = np.array(xp0)
    if xc0 is None:
        xc0 = np.zeros((0,))
    else:
        xc0 = np.array(xc0)
    ts = np.array(ts)
    noise = np.array(noise)

    if not sohc.is_discrete_time:
        raise ETCSimError('Simulation only ready for Periodic ETC')
    if sohc.is_discrete_time:
        if sohc.h != sohc.controller.h:
            raise ETCSimError('Simulation requires steps \'h\''
                              ' of PETC and controller to be the same:')
        ratios = sohc.h / np.diff(ts)
        if not (np.abs(ratios - np.round(ratios)) < 1e-8*max(ratios)).all():
            warnings.warn('ts breakpoints must be multiples of sohc.h',
                          ETCSimWarning)
            ts = np.arange(ts[0], ts[-1], sohc.h)

    steps = np.size(ts)

    if disturbance is None:
        def disturbance(t): return np.zeros((p.nw,))

    if np.size(noise) == 1:
        noise = np.tile(noise, (p.ny, steps))

    # Initialize
    xp = xp0
    xc = xc0
    xs = xs0
    xphat = xp
    xchat = xc
    y = p.measurement(xphat, noise[:, 0])
    yhat = y
    u = c.output(yhat, xchat)
    uhat = u

    # Initialize storage
    xps = np.zeros((p.nx, steps)) * np.nan
    xphats = np.zeros((p.nx, steps)) * np.nan
    xcs = np.zeros((c.nx, steps)) * np.nan
    xchats = np.zeros((c.nx, steps)) * np.nan
    ys = np.zeros((p.ny, steps)) * np.nan
    yhats = np.zeros((p.ny, steps)) * np.nan
    us = np.zeros((p.nu, steps)) * np.nan
    uhats = np.zeros((p.nu, steps)) * np.nan
    ws = np.zeros((p.nw, steps)) * np.nan
    samples = np.zeros((steps,), dtype=bool)  # True if sampled

    stop = False  # May stop if some error occurs

    if scheduler_has_states:
        nxs = len(xs)
        xss = np.zeros((nxs, steps))

    # Make ode callables
    def fp(t, x): return sohc.fp(t, x, uhat, disturbance)
    def fc(t, x): return sohc.fc(t, x, yhat)

    pint = integrate.ode(fp).set_integrator('dopri5')
    pint.set_initial_value(xp, ts[0])
    if not c.is_discrete_time and c.is_dynamic:
        cint = integrate.ode(fp).set_integrator('dopri5')
        cint.set_initial_value(xc, ts[0])
    else:
        cint = None

    def updatestates(t, dt, pint, c, cint, fc, xc, yhat):
        xp = pint.integrate(t)
        if not c.is_discrete_time and c.is_dynamic:
            xc = cint.integrate(t)
        elif c.is_dynamic:
            # FIXME: Only works if step is equal to h
            xc = fc(t, xc, yhat)
        return xp, xc

    # Intial updates
    i = 0

    xps[:, i] = xp
    xphats[:, i] = xphat
    xcs[:, i] = xc
    xchats[:, i] = xchat
    ys[:, i] = y
    yhats[:, i] = yhat
    us[:, i] = u
    uhats[:, i] = uhat
    ws[:, i] = disturbance(ts[i])

    if scheduler_has_states:
        xss[:, i] = xs

    t = ts[0]
    dt = 0
    if sohc._sampling_type == 'STC':
        if scheduler_has_states:
            dt_star, xs = sohc.sampling_time(xp, xc, xs=xs)
        else:
            dt_star = sohc.sampling_time(xp, xc, xs=xs)
    samples[i] = True
    sample = False

    for i, t in enumerate(ts[1:]):
        if sohc.is_discrete_time:
            dt += t - ts[i]
#            if t > ts[i]:  # Break point set in-between discrete ETC times
#                told = t
#                t = ts[i]
#
#                xp, xc = updatestates(t, dt, pint, c, cint, fc, yhat)
#                y = p.measurement(xp, noise[i])
#                u = c.output(y, xc)
#
#                # Update stuff
#                xps[:,i] = xp
#                xphats[:,i] = xphat
#                xcs[:,i] = xc
#                xchats[:,i] = xchat
#                ys[:,i] = y
#                yhats[:,i] = yhat
#                us[:,i] = u
#                uhats[:,i] = uhat
#                ws[:,i] = disturbance(ts[i])
#
#                # Go back to the original time
#                t = told
#            #end if t > ts[i]

            # Here I am in a discrete ETC time
            xp, xc = updatestates(t, dt, pint, c, cint, fc, xc, yhat)

            # Condition checking: for ETC, trigger; for STC, time
            k = t/sohc.h
            if np.abs(k - round(k)) < 1e-10:
                y = p.measurement(xp, noise[:, i])
                u = c.output(y, xc)
                if sohc._sampling_type == 'ETC':
                    sample = sohc.trigger(dt, xp, xphat, y, yhat, u, uhat, t)
                elif sohc._sampling_type == 'STC':
                    sample = dt >= dt_star - sohc.h/2
                elif sohc._sampling_type == 'Periodic':
                    sample = True
                else:
                    raise ETCSimError('input must be either ETC, STC,'
                                   ' or periodic')

                if sample:
                    xphat = xp
                    xchat = xc
                    uhat = u
                    yhat = y
                    dt = 0.0
                    if sohc._sampling_type == 'STC':
                        try:
                            if scheduler_has_states:
                                dt_star, xs = sohc.sampling_time(xp, xc, xs=xs)
                            else:
                                dt_star = sohc.sampling_time(xp, xc)
                        except ETCSimError as e:
                            logging.debug(str(e))
                            stop = True

            xps[:, i+1] = xp
            xphats[:, i+1] = xphat
            xcs[:, i+1] = xc
            xchats[:, i+1] = xchat
            ys[:, i+1] = y
            yhats[:, i+1] = yhat
            us[:, i+1] = u
            uhats[:, i+1] = uhat
            ws[:, i+1] = disturbance(t)
            samples[i+1] = sample

            sample = False

            if stop:
                break
        # end if sohc.is_discrete_time
        # TODO: else
    # end for i in range(1,ts.size)

    # Return dictionary
    return {'t': ts, 'xp': xps, 'xc': xcs, 'y': ys, 'u': us, 'xphat': xphats,
            'xchat': xchats, 'yhat': yhats, 'uhat': uhats, 'w': ws,
            'sample': samples}


'''
     TESTS
'''

if __name__ == '__main__':
    # with open('./tests/scenarios/simple.py') as f:
    #     code = compile(f.read(), "simple.py", 'exec')
    #     exec(code)  # , global_vars, local_vars)

    # h = 0.05
    # plant = LinearPlant(Ap, Bp, Cp, None, E)
    # controller = LinearController(K, h)
    # trig = TabuadaPETC(plant, controller, None, None, sigma)
    # trig = RelaxedPETC(plant, controller, Pl, lbd, kmin=8)
    # # trig.sigma = 0.2

    # x0 = np.array([0.7, -0.21])
    # x0 = np.array([1, 1])
    # x0 = np.array([1, -1])
    # trig.kmax = 20
    # t0, t1 = 0, 10
    # t = np.arange(t0, t1, 0.01)

    # with open('./tests/scenarios/batch_reactor.py') as f:
    #     code = compile(f.read(), "batch_reactor.py", 'exec')
    #     exec(code)  # , global_vars, local_vars)

    from tests.scenarios.batch_reactor import Ap, Bp, Cp, K, Pl, lbd

    h = 0.01
    plant = etc.LinearPlant(Ap, Bp, Cp)
    controller = etc.LinearController(K, h)
    trig = etc.RelaxedPETC(plant, controller, Pl, lbd, kmin=8)

    x0 = np.array([-1, 1, -1, 1])
    trig.kmax = 20
    t0, t1 = 0, 2
    t = np.arange(t0, t1, 0.01)

    w = None  # lambda t: np.array([0*np.sin(t)])
    out = simulate_sample_and_hold_control(trig, t, x0, [],
                                           disturbance=w)

    if 1:
        import matplotlib.pyplot as plt

        # Plots!
        plt.close('all')
        plt.plot(t, out['xp'].T, t, out['xphat'].T)
        plt.show()

        plt.figure()
        plt.plot(t, out['uhat'].T)
        plt.show()

        trigs = (out['xp'] == out['xphat']).all(0)
        # plt.figure()
        # plt.stem(t, trigs)
        # plt.show()

        ttrig = t[trigs]
        dttrig = np.diff(ttrig)
        plt.figure()
        plt.plot(ttrig[1:], np.round(dttrig/trig.h), '*')
        plt.show()

    ''' Periodic '''
    periodic = etc.PeriodicLinearController(plant, controller, 0.08)
    outp = simulate_sample_and_hold_control(periodic, t, x0, [],
                                            disturbance=w)

    if 1:
        import matplotlib.pyplot as plt

        # Plots!
        plt.figure()
        plt.plot(t, outp['xp'].T, t, outp['xphat'].T)
        plt.show()

        plt.figure()
        plt.plot(t, outp['uhat'].T)
        plt.show()


    ''' Almost continuous '''
    periodic = etc.PeriodicLinearController(plant, controller, 0.01)
    outp = simulate_sample_and_hold_control(periodic, t, x0, [],
                                            disturbance=w)

    if 1:
        import matplotlib.pyplot as plt

        # Plots!
        plt.figure()
        plt.plot(t, outp['xp'].T, t, outp['xphat'].T)
        plt.show()

        plt.figure()
        plt.plot(t, outp['uhat'].T)
        plt.show()
