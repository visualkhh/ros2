from . import Kros
from .nodes import Node

kros = Kros(lambda ctx: [Node(ctx, "hello")])
print(kros.nodes)
