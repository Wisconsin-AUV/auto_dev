import py_trees

def gateSubTree():
#basic gateSubTree, will have similar tree for pole. Mostly just brainstorming
    gateSubTree = py_trees.composites.Sequence(name="Gate SubTree", memory=True)
    moveToGate = py_trees.composites.Sequence(name="Move to Gate", memory=False)
    completeGate = py_trees.composites.Sequence(name="Complete Gate", memory=True)


#no actions coded yet, placeholder words just for tree frame
    moveToGate.add_children([detectGate, alignWithGate])
    completeGate = add_children([rollThroughGate, markGateDone])
    gateSubTree.add_children([moveToGate, completeGate])