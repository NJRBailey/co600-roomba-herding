# HierarchyReader
# Takes a list of contours and returns a collection of linkedLists to show the hierarchies
# of the nested contours.


## Node class represents a single contour.
class Node:

    ## Initialises a Node.
    #
    # @param ident The identifier for this contour.
    # @param parent The identifier for the parent contour.
    # @param firstChild The identifier for the first child contour.
    def __init__(self, ident, parent, firstChild):
        self.id = ident
        self.parent = parent
        self.firstChild = firstChild

    ## Prints this Node's attributes in the terminal.
    def printSelf(self):
        print('id: ' + str(self.id) + ' | parent: ' + str(self.parent) + ' | first child: ' + str(self.firstChild))


## LinkedList class holds all the nested contours for each connected contour.
class LinkedList:
    ## Initialises a LinkedList.
    #
    # @param lowestNode The lowest node in the hierarchy, i.e. the node with most ancestors.
    # @param nodesList A list containing all nodes.
    def __init__(self, lowestNode, nodesList):
        self.list = [lowestNode]
        # Loop up through the parents until we find the top
        while self.list[0].parent != -1:
            parentNode = 0
            for node in nodesList:
                if node.id == self.list[0].parent:
                    parentNode = node
            self.list.insert(0, parentNode)  # TODO use collections.deque for quicker adding

    ## Prints this list's Node IDs in the terminal.
    def printList(self):
        idsList = []
        for node in self.list:
            idsList.append(node.id)
        print(idsList)


## Constructs and returns an array of linkedLists to represent the hierarchy of each contour.
#
# @param contourList A list of OpenCV contours.
# @return A list of LinkedList objects.
def readHierarchy(contourList):
    nodesList = []
    linkedLists = []
    for index in range(len(contourList)):
        # Make new Node
        nodesList.append(Node(index, contourList[index][3], contourList[index][2]))
    lowestNodes = []
    for node in nodesList:
        if node.firstChild == -1:
            lowestNodes.append(node)
    for lowNode in lowestNodes:
        linkedLists.append(LinkedList(lowNode, nodesList))
    return linkedLists
