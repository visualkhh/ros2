from . import Kros
from .node import Node

kros = Kros(lambda ctx: [Node(ctx, "hello")])
print(kros.nodes)
