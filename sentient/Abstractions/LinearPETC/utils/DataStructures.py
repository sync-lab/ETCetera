from collections import namedtuple


Partition = namedtuple('Partition', ['k', 'v'])


PTA = namedtuple('PTA', ('Locations', 'InitialLocations', 'Actions',
                         'Clocks', 'Edges', 'Invariants', 'Prices',
                         'LocationPrices', 'EdgePrices'))