# Contours hierarchy is an accessible list
# hierarchy[0][x] will get you the xth contour
# so loop through 'x = 1 to N' in hierarchy[0][x] and group nodes

# Next, Prev, FirstChild, Parent
# Next contour at the same hierarchical level
# Previous contour at the same hierarchical level
# First child contour
# Parent contour


class Node:
    def __init__(self, ident, parent, firstChild):
        self.id = ident
        self.parent = parent
        self.firstChild = firstChild

    def printSelf(self):
        print('id: ' + str(self.id) + ' | parent: ' + str(self.parent) + ' | first child: ' + str(self.firstChild))


class LinkedList:
    def __init__(self, lowestNode, nodesList):
        self.list = [lowestNode]
        # Loop up through the parents until we find the top
        # print('printing self.list[0]')
        # print(self.list[0])
        while self.list[0].parent != -1:
            parentNode = 0
            for node in nodesList:
                if node.id == self.list[0].parent:
                    parentNode = node

            self.list.insert(0, parentNode)  # todo use collections.deque for quicker adding

        # for node in self.list:
            # print(node.id)

    def printList(self):
        idsList = []
        for node in self.list:
            idsList.append(node.id)

        print(idsList)


def readHierarchy(contourList):
    nodesList = []
    linkedLists = []

    for index in range(len(contourList)):
        # Make new Node
        nodesList.append(Node(index, contourList[index][3], contourList[index][2]))

    # print('SUCCESSFUL LOOP')

    lowestNodes = []
    for node in nodesList:
        # Print information about Node
        # node.printSelf()

        if node.firstChild == -1:
            lowestNodes.append(node)

    for lowNode in lowestNodes:
        linkedLists.append(LinkedList(lowNode, nodesList))

    # for ll in linkedLists:
        # if len(ll.list) > 2:
        # ll.printList()

    # print('SUCCESSFUL TERMINATION')

    return linkedLists
