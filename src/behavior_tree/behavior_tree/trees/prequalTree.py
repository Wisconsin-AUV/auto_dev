import py_trees

#add actions
from blackboard import client, prequalTasks

#Actions that can be merged into one
#   gateCheckBoard/poleCheckBoard
#   lookAround/alignGate/rollGate

#Actions not needed (shouldnt vision bools update every tick independent of tree)
#   updateGate/updatePole


#Subtree for the gate task: find the gate, line up, drive through, mark done
def getGateSubtree():
    #check blackboard for gate action (gateCheckBoard)

    #sub look around action (lookAround)

    #update gate boolean from vision action (updateGate)

    #gate alignment action (alignGate)

    #gate roll action (rollGate)

    #mark task complete action with param task (markTaskComplete("gate"))

    #skip search if already found
    searchGate = py_trees.composites.Selector("Gate Skip", False)

    searchSequence = py_trees.composites.Sequence("Search and Update", True)

    searchSequence.add_children(
        [lookAround, updateGate]
    )

    searchGate.add_children(
        [gateCheckBoard, searchSequence]
    )

    gateTree = py_trees.composites.Sequence("Gate Subtree", True)

    gateTree.add_children(
        [searchGate, alignGate, rollGate, markTaskComplete("gate")]
    )

    return gateTree


#Subtree for the pole task, same shape as the gate subtree with pole names/flag
def getPoleSubtree():
    #check blackboard for pole vision bool (poleCheckBoard)

    #sub look around action (lookAround)

    #update pole boolean from vision action (updatePole)

    #go to the pole action (goToPole)

    #go around pole action (goAroundPole)

    #mark task complete action with param task (markTaskComplete("pole"))

    #skip search if already found
    searchPole = py_trees.composites.Selector("Pole Skip", False)

    searchSequence = py_trees.composites.Sequence("Search and Update", True)

    searchSequence.add_children(
        [lookAround, updatePole]
    )

    searchPole.add_children(
        [poleCheckBoard, searchSequence]
    )

    poleTree = py_trees.composites.Sequence("Pole Subtree", True)

    poleTree.add_children(
        [searchPole, goToPole, goAruondPole, markTaskComplete("pole")]
    )

    return poleTree


#Subtree for the return task: head home and surface
def create_return_subtree():
    #check blackboard for home vision bool (homeCheckBoard)

    #sub look around action (lookAround)

    #update home boolean from vision action (updateHome)

    #turn and go home action (goHome)

    #mark task complete action with param task (markTaskComplete("return"))

    #skip search if already found
    searchHome = py_trees.composites.Selector("Home Skip", False)

    searchSequence = py_trees.composites.Sequence("Search and Update", True)

    searchSequence.add_children(
        [lookAround, updateHome]
    )

    searchHome.add_children(
        [homeCheckBoard, searchSequence]
    )

    homeTree = py_trees.composites.Sequence("Pole Subtree", True)

    homeTree.add_children(
        [searchHome, goHome, markTaskComplete("return")]
    )

    return homeTree


#Builds the detailed prequal loop: same skeleton as the competition tree, but the
#perform step dispatches into one of the subtrees above.
def getPrequalTree():
    #pop task from queue to currentTask (popTask)

    #action to choose what tree (chooseTree)

    #action to perform task (doTask("taskName"))

    #action to check if a task is done (isTaskDone)

    #Reset the blackboard
    client.taskQueue = prequalTasks.copy()
    client.currentTask = None
    client.completedTasks = []
    client.gateFound = False
    client.poleFound = False
    client.homeFound = False

    #Skip if possible
    skipOrDo = py_trees.composites.Selector("Skip Task or Do Task", False)
    skipOrDo.add_children(
        [isTaskDone, doTask]
    )

    getAndDoTask = py_trees.composites.Sequence("Get And Do Task", True)
    getAndDoTask.add_children(
        [popTask, skipOrDo]
    )

    #Repeat forever until popTask fails on an empty queue
    root = py_trees.decorators.Repeat(
        child=getAndDoTask,
        num_success=-1,
        name="Repeat Prequal"
    )

    return root
