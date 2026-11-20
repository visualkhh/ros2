from . import Kros
from .node import Node

kros = Kros(lambda ctx: [Node("hello", ctx)])
print(kros.nodes)
