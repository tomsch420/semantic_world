from typing import Dict, Callable, Union, Tuple, List

import numpy as np

from . import spatial_types as cas

Provider = Union[float, Callable[[], float]]

from threading import Lock


class SingletonMeta(type):
    """
    This is a thread-safe implementation of Singleton.
    Source: https://refactoring.guru/design-patterns/singleton/python/example#example-1
    """

    _instances = {}

    _lock: Lock = Lock()

    def __call__(cls, *args, **kwargs):
        """
        Possible changes to the value of the `__init__.py` argument do not affect
        the returned instance.
        """
        with cls._lock:
            if cls not in cls._instances:
                instance = super().__call__(*args, **kwargs)
                cls._instances[cls] = instance
        return cls._instances[cls]


class SymbolManager(metaclass=SingletonMeta):
    """
    Singleton because I don't want multiple symbols with the same name to refer to different objects.
    TODO usually called registry
    Manages the association of symbolic variables with their providers and facilitates operations on them.

    The `SymbolManager` class is a tool for managing symbolic variables and their associated value providers.
    It allows the registration of various mathematical entities such as points, vectors, quaternions, and
    provides methods for resolving these symbols to their numeric values. The class also supports the
    evaluation of expressions involving these symbols.

    The purpose of this class is to abstract the management of symbolic variables and enable their seamless
    use in mathematical and symbolic computations.
    """
    symbol_to_provider: Dict[cas.Symbol, Callable[[], float]]
    """
    A dictionary mapping symbolic variables (`cas.Symbol`) to callable functions that provide numeric values for those symbols.
    """

    def __init__(self):
        self.symbol_to_provider = {}

    def register_symbol_provider(self, name: str, provider: Provider) -> cas.Symbol:
        symbol = cas.Symbol(name)
        self.symbol_to_provider[symbol] = provider
        return symbol

    def register_point3(self, name: str, provider: Callable[[], Tuple[float, float, float]]) -> cas.Point3:
        sx = self.register_symbol_provider(f'{name}.x', lambda: provider()[0])
        sy = self.register_symbol_provider(f'{name}.y', lambda: provider()[1])
        sz = self.register_symbol_provider(f'{name}.z', lambda: provider()[2])
        p = cas.Point3([sx, sy, sz])
        return p

    def register_vector3(self, name: str, provider: Callable[[], Tuple[float, float, float]]) -> cas.Vector3:
        sx = self.register_symbol_provider(f'{name}.x', lambda: provider()[0])
        sy = self.register_symbol_provider(f'{name}.y', lambda: provider()[1])
        sz = self.register_symbol_provider(f'{name}.z', lambda: provider()[2])
        v = cas.Vector3([sx, sy, sz])
        return v

    def register_quaternion(self, name: str, provider: Callable[[], Tuple[float, float, float, float]]) \
            -> cas.Quaternion:
        sx = self.register_symbol_provider(f'{name}.x', lambda: provider()[0])
        sy = self.register_symbol_provider(f'{name}.y', lambda: provider()[1])
        sz = self.register_symbol_provider(f'{name}.z', lambda: provider()[2])
        sw = self.register_symbol_provider(f'{name}.w', lambda: provider()[3])
        q = cas.Quaternion((sx, sy, sz, sw))
        return q

    def register_transformation_matrix(self, name: str,
                                       provider: Callable[[], Tuple[Tuple[float, float, float, float],
                                                                    Tuple[float, float, float, float],
                                                                    Tuple[float, float, float, float]]]) \
            -> cas.TransformationMatrix:
        symbols = []
        for row in range(3):
            symbols.append([])
            for col in range(4):
                symbols[row].append(self.register_symbol_provider(f'{name}[{row},{col}]',
                                                                  lambda r=row, c=col: provider()[r][c]))
        symbols.append([0,0,0,1])
        root_T_tip = cas.TransformationMatrix(symbols)
        return root_T_tip

    def resolve_symbols(self, symbols: Union[List[cas.Symbol], List[List[cas.Symbol]]]) \
            -> Union[np.ndarray, List[np.ndarray]]:
        try:
            if len(symbols) == 0:
                return np.array([])
            if isinstance(symbols[0], list):
                return [np.array([self.symbol_to_provider[s]() for s in param], dtype=float) for param in symbols]
            else:
                return np.array([self.symbol_to_provider[s]() for s in symbols], dtype=float)
        except Exception as e:
            # Flatten symbols for error checking
            flattened_symbols = []
            if len(symbols) > 0 and isinstance(symbols[0], list):
                # symbols is a list of lists
                for sublist in symbols:
                    flattened_symbols.extend(sublist)
            else:
                # symbols is already a flat list
                flattened_symbols = symbols

            for s in flattened_symbols:
                try:
                    np.array([self.symbol_to_provider[s]()])
                except Exception as e2:
                    raise KeyError(f'Cannot resolve {s} ({e2.__class__.__name__}: {str(e2)})')
            raise e

    def resolve_expr(self, expr: cas.CompiledFunction):
        return expr.fast_call(*self.resolve_symbols(expr.symbol_parameters))

    def evaluate_expr(self, expr: cas.Expression):
        if isinstance(expr, (int, float)):
            return expr
        f = expr.compile()
        if len(f.symbol_parameters) == 0:
            return expr.to_np()
        result = f.fast_call(*self.resolve_symbols(f.symbol_parameters))
        if len(result) == 1:
            return result[0]
        else:
            return result


symbol_manager = SymbolManager()
