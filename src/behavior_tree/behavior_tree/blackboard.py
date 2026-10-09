import py_trees

#TODO: comp tasks/comp vision bools
compTasks = []
prequalTasks = ["gate", "pole", "return"]

client = py_trees.blackboard.Client(name="Global")

#universals
client.register_key("taskQueue", access=py_trees.common.Access.WRITE)
client.register_key("currentTask", access=py_trees.common.Access.WRITE)
client.register_key("completedTasks", access=py_trees.common.Access.WRITE)

client.taskQueue = prequalTasks.copy()
client.currentTask = None
client.completedTasks = []

#prequal specific
client.register_key("gateFound", access=py_trees.common.Access.WRITE)
client.register_key("poleFound", access=py_trees.common.Access.WRITE)
client.register_key("homeFound", access=py_trees.common.Access.WRITE)

client.gateFound = False
client.poleFound = False
client.homeFound = False

#TODO: comp specific