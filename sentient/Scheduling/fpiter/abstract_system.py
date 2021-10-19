from abc import ABCMeta, abstractmethod
from typing import List, Tuple
import logging



class abstract_system(metaclass=ABCMeta):
    """
    Abstract class representing a composition of multiple transition systems abstracting the triggering behaviour of
    PETC control loops.
    """

    def __init__(self, cl):
        self.control_loops = cl
        self.ns = len(cl)
        self.h = cl[0].h


    """ Partitioning and Refinement Methods """
    def partition_all(self):
        success = False
        for cl in self.control_loops:
            success |= cl.create_initial_partition()

        return success

    def refine_all(self):
        success = False
        for cl in self.control_loops:
            success |= cl.refine()

        return success

    def partition(self, idx: list):
        """
        Partitions systems in the list
        :param idx: List of subsystem numbers
        :return: Whether partitioning is successful
        """
        if any([x >= self.ns for x in idx]):
            print("One or more specified indices out of range.")
            return False

        s = [i not in idx for i in range(0, self.ns)]
        for i in range(0, self.ns):
            s[i] = self.control_loops[i].create_initial_partition()

        return all(s)

    def refine(self, idx: list):
        if any([x >= self.ns for x in idx]):
            print("One or more specified indices out of range.")
            return False

        success = False
        for n in idx:
            success |= self.control_loops[n].refine()

        return success

    def restore_all(self):
        for cl in self.control_loops:
            cl.restore()

        return True

    def restore(self, idx: list):
        if any([x >= self.ns for x in idx]):
            print("One or more specified indices out of range.")
            return False

        for n in idx:
            self.control_loops[n].restore()

        return True

    """ Abstract Methods """
    @abstractmethod
    def compose(self):
        raise NotImplementedError

    @abstractmethod
    def safe_set(self):
        raise NotImplementedError

    @abstractmethod
    def safety_game(self, W=None) -> Tuple[object, bool]:
        raise NotImplementedError

    @abstractmethod
    def create_controller(self, Z, StatesOnlyZ=True, convert_blocks=True):
        raise NotImplementedError

    # @abstractmethod
    # def _block2states(self, C):
    #     raise NotImplementedError

    """ Scheduling Algorithms """
    def generate_safety_scheduler(self):
        """
        Will choose one of the safety scheduling algorithms. By default will choose the partitioning_all strategy,
        but each system implementation can/should overwrite this to dynamically choose the most optimal one.
        @return: Resulting scheduler
        """
        return self.gen_safety_scheduler_part()

    def gen_safety_scheduler_basic(self):
        """

        @return:
        """
        print("Starting basic scheduler synthesization.")
        logging.info("Composing system.")
        self.compose()

        logging.info("Generating safe set.")
        W = self.safe_set()
        if W is None:
            print("No safe set found.")
            return None

        logging.info("Solving safety game.")
        Z = self.safety_game(W)
        if Z is None:
            print("No solution to safety game found.")
            return None, None

        logging.info("Generating scheduler.")
        C, Q = self.create_controller(Z, StatesOnlyZ=False)
        print('Scheduler found!')
        self.scheduler = C
        self.state2block = None
        return C, Q

    def gen_safety_scheduler_part(self, convert_blocks=False):
        """
            Tries to synthesize a scheduler somewhat more efficiently by reducing the size of all subsystem,
            and synthesizing on the composition of those. If it fails, it will refine each subsystem and
            try again, until refinement is no longer possible.
            :param S:
            :return:
            """
        print("Starting scheduler synthesization with partitioning.")
        logging.info("Partitioning all subsystems.")
        res = self.partition_all()
        if not res:
            print("Partitioning failed.")
            return None

        num_tries = 1
        RefSuccess = True
        # Start synthesization loop
        while RefSuccess:
            logging.info("Try: {n}".format(n=num_tries))
            logging.info("Composing system.")
            self.compose()

            logging.info("Generating safe set.")
            W = self.safe_set()
            if W is None:
                print("No safe set found.")
                return None

            logging.info("Solving safety game.")
            Z = self.safety_game(W)
            if Z is None:
                tempSuccess = False
                logging.info("No solution to safety game found.")
                logging.info("Refine all subsystem and try again..")
                # while RefSuccess:
                RefSuccess = self.refine_all()
                # tempSuccess = True
                num_tries += 1
                continue

            logging.info('Safety game solved!')
            logging.info('Generating scheduler.')
            C, Q = self.create_controller(Z, StatesOnlyZ=False, convert_blocks=convert_blocks)
            print('Scheduler found!')
            self.scheduler = C
            self.state2block = Q
            return C, Q

        print('Refinement no longer possible.')
        print('No scheduler found.')

        return None, None