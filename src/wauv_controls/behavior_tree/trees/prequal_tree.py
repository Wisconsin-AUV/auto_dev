import py_trees

def gateSubTree():
#basic gateSubTree, will have similar tree for pole. Mostly just brainstorming
    GateSubTree = py_trees.composites.Selector(name="Pole SubTree", memory=True)
    gateSubTreeActions = py_trees.composites.Sequence(name="Gate SubTree Actions", memory=True)
    moveToGate = py_trees.composites.Sequence(name="Move To Gate", memory=True)
    completeGate = py_trees.composites.Sequence(name="Complete Gate", memory=True)


#no actions coded yet, placeholder words just for tree frame
    moveToGate.add_children([detectGate, alignWithGate])
    completeGate.add_children([rollThroughGate, markGateDone])
    gateSubTreeActions.add_children([moveToGate, completeGate])
    GateSubTree.add_children([isPoleDone, poleSubTreeActions])


def poleSubTree():
    poleSubTree = py_trees.composites.Selector(name="Pole SubTree", memory=True)
    poleSubTreeActions = py_trees.composites.Sequence(name="Pole SubTree Actions", memory=True)
    moveToPole = py_trees.composites.Sequence(name="Move To Pole", memory=True)
    completePole = py_trees.composites.Sequence(name="Complete Pole", memory=True)

    moveToPole.add_children([detectPole, alignWithPole])
    completePole.add_children([moveAroundPole, markPoleDone])
    poleSubTreeActions.add_children([moveToPole, completePole])
    poleSubTree.add_children([isPoleDone, poleSubTreeActions])

def returnSubTree():
    returnSubTree = py_trees.composites.Sequence(name="Return SubTree", memory=True)
    tasksCompleted = py_trees.composites.Sequence(name="Tasks Completed", memory=True)
    returnHome = py_trees.composites.Selector(name="Return Home", memory=True)

    returnHome.add_children([detectStartingPos, moveToHome])
    tasksCompleted.add_children([isGateDone, isPoleDone])
    returnSubTree.add_children([tasksCompleted, returnHome])