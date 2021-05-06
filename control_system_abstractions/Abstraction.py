import pickle
import os
from abc import ABCMeta, abstractmethod
from functools import lru_cache

from config import save_path

from linear_systems_utils.linearetc import ETC
from timed_automaton import TimedAutomaton




class InputDataStructure:
    """
        Abstract data structure to be used as input for different traffic abstraction algorithms
    """
    pass


class Abstraction(metaclass=ABCMeta):
    """ Abstract class """

    # plant: Plant
    # controller: Controller
    ETC: ETC

    def __init__(self, inputdata: InputDataStructure):
        pass

    @classmethod
    def from_bytestream_file(cls, file_name):
        if not os.path.isfile(file_name):
            file_retry = os.path.join(save_path, file_name)
            if not os.path.isfile(file_retry):
                print("Please specify a valid file.")
                return None
            else:
                file_name = file_retry

        with open(file_name, 'rb') as f:
            obj = pickle.load(f)

        return obj

    @abstractmethod
    def construct_regions(self, *args, **kwargs) -> None:
        """
        Constructs a partitioning of the state space of the given ETC,
        depending on the specific algorithm that implements this class.
        """
        raise NotImplementedError

    @abstractmethod
    def create_abstraction(self, *args, **kwargs) -> None:
        """
        Creates the complete traffic abstraction. If the regions are not
        constructed yet, the method self.construction_regions() must be called
        beforehand.
        """
        raise NotImplementedError

    @abstractmethod
    def refine(self) -> None:
        """
        Refines the abstraction is some way. Details will depend on the algorithm
        implementing this class.
        This must be called last in any overwriting methods.
        """

        # Invalidate the cached timed automaton
        Abstraction.timed_automaton.fget.cache_clear()

    """ Creation and updating of timed automata """

    @property
    @lru_cache()
    def timed_automaton(self) -> TimedAutomaton:
        """
        Creates a Timed Automaton
        """
        ta_data = self._traffic2ta()
        return TimedAutomaton(*ta_data, abstraction=self)

    @abstractmethod
    def _traffic2ta(self) -> list:
        """
        Converts the abstraction to a list of parameters containing the:
        states, initial_states, clocks, invariants, guards, actions and transitions
        """
        raise NotImplementedError

    """ Exportation """

    def export(self, file_name: str = None, export_type: str = 'pickle'):
        export_type = export_type.lower()
        if file_name is None:
            file_name = self.__class__.__name__
        if export_type in ['pickle', 'bytes', 'byte_stream']:
            self.__export_pickle(file_name)

    def __export_pickle(self, file_name: str):
        with open(os.path.join(save_path, file_name), 'wb') as f:
            pickle.dump(self, f, pickle.HIGHEST_PROTOCOL)
