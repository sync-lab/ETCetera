import pickle, json
import os
from abc import ABCMeta, abstractmethod
from functools import cached_property

from config import save_path

from ETCetera.Systems.Automata import TimedAutomaton, Automaton

class Abstraction(metaclass=ABCMeta):
    """ Abstract class """

    # plant: Plant
    # controller: Controller
    # ETC: ETC

    def __init__(self, *args, **kwargs):
        pass

    # @abstractmethod
    # def generate_regions(self) -> None:
    #     """
    #     Constructs a partitioning of the state space of the given ETC,
    #     depending on the specific algorithm that implements this class.
    #     """
    #     raise NotImplementedError
    #
    # @cached_property
    # def regions(self):
    #     return self.generate_regions()
    #
    # # @abstractmethod
    # # def generate_transitions(self):
    #
    # @abstractmethod
    # def create_abstraction(self, *args, **kwargs) -> None:
    #     """
    #     Creates the complete traffic abstraction. If the regions are not
    #     constructed yet, the method self.construction_regions() must be called
    #     beforehand.
    #     """
    #     raise NotImplementedError
    #
    # @abstractmethod
    # def refine(self) -> None:
    #     """
    #     Refines the abstraction is some way. Details will depend on the algorithm
    #     implementing this class.
    #     This must be called last in any overwriting methods.
    #     """
    #
    #     # Invalidate the cached timed automaton
    #     #Abstraction.timed_automaton.fget.cache_clear()
    #     # if 'timed_automaton' in self.__dict__:
    #     #     del self.__dict__['timed_automaton']
    #     pass


    """  Methods to create a automata """

    @abstractmethod
    def _create_automaton(self) -> Automaton:
        raise NotImplementedError

    @cached_property
    def automaton(self):
        return self._create_timed_automaton()

    """ Methods to create a timed automata """

    @abstractmethod
    def _create_timed_automaton(self) -> TimedAutomaton:
        raise NotImplementedError

    @cached_property
    def timed_automaton(self) -> TimedAutomaton:
        """
        Creates a Timed Automaton
        """
        return self._create_timed_automaton()

    """ Exportation """

    def export(self, file_name: str = None, export_type: str = 'pickle'):
        export_type = export_type.lower()
        if file_name is None:
            file_name = self.__class__.__name__
        if export_type in ['pickle', 'bytes', 'byte_stream']:
            self._export_pickle(file_name)
        elif export_type in ['json']:
            self._export_json(file_name)

    def _export_pickle(self, file_name: str):
        if not file_name.endswith('.pickle'):
            file_name += '.pickle'
        with open(os.path.join(save_path, file_name), 'wb') as f:
            pickle.dump(self, f, pickle.HIGHEST_PROTOCOL)

    def _export_json(self, file_name:str):
        if not file_name.endswith('.json'):
            file_name += '.json'
        with open(os.path.join(save_path, file_name), 'w') as f:
            json.dump(self.__repr__(), f)

    def _export_txt(self, file_name:str):
        if not file_name.endswith('.txt'):
            file_name += '.json'
        with open(os.path.join(save_path, file_name), 'w') as f:
            f.write(str(self.__repr__()))

    @classmethod
    def from_bytestream_file(cls, file_name) -> 'Abstraction':
        file_name = cls._check_file(file_name)
        if file_name is None:
            raise FileNotFoundError

        with open(file_name, 'rb') as f:
            obj = pickle.load(f)

        return obj

    @abstractmethod
    def __repr__(self):
        raise NotImplementedError

    # @classmethod
    # def from_json(cls, file_name):
    #     # Check if file exists.
    #     # If not check if it is because the save path is not specified
    #     file_name = cls._check_file(file_name)
    #     if file_name is None:
    #         return None
    #
    #     with open(file_name, 'r') as f:
    #         obj = json.load(f)
    #
    #     # This feels a bit like cheating....
    #     temp = cls(**obj)
    #     temp.__dict__.update(obj)
    #     return temp

    @staticmethod
    def _check_file(file_name):
        """ Check if file exists. If not check if it is because the save path is not specified.
        If it exists return the file_name, else None"""
        if not os.path.isfile(file_name):
            file_retry = os.path.join(save_path, file_name)
            if not os.path.isfile(file_retry):
                print("Please specify a valid file.")
                return None
            else:
                file_name = file_retry

        return file_name
