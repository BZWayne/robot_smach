#!/usr/bin/env python

"""
The map module for topological_map.
It includes three submodules for commands:
1. Manipulation - commands that can modify information stored in an ontology
2. Query - commands used to retrieve information from the ontology
3. Sysutil - utility commands such as load/save ontology and toggle ARMOR logging
Also includes:
1. Exceptions - All exceptions that armor_api commands can raise
"""

from utilities import armor_manipulation_client, armor_query_client, armor_utils_client, armor_exceptions, armor_client

__author__ = "Alessio Capitanelli"
__copyright__ = "Copyright 2016, ArmorPy"
__license__ = "GNU"
__version__ = "1.0.0"
__maintainer__ = "Alessio Capitanelli"
__email__ = "alessio.capitanelli@dibris.unige.it"
__status__ = "Development"

__all__ = ['ArmorManipulationClient.py', 'ArmorQueryClient.py', 'ArmorUtilsClient.py', 'ArmorExceptions.py', 'ArmorClient.py']
