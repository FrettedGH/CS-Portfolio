from sys import argv
import json
import heapq

from PySide6.QtWidgets import QApplication, QLabel, QStatusBar, QGraphicsLineItem, QGraphicsPathItem, QMenu, QGraphicsObject, QGraphicsEllipseItem, QStackedWidget, QGraphicsItem, QPushButton, QHBoxLayout, QGraphicsView, QGraphicsScene, QGraphicsRectItem, QMainWindow, QVBoxLayout, QWidget, QButtonGroup, QSizePolicy, QGraphicsTextItem, QToolBar, QGridLayout, QInputDialog
from PySide6.QtCore import Qt, Signal, QPoint, QRectF, QPointF, QLineF, QTimer
from PySide6.QtGui import QAction, QBrush, QColor, QPen, QPainter, QFont, QPainterPath, QTransform
from PySide6.QtSvg import QSvgRenderer

from numpy import linalg, zeros
from math import inf

#---------------------------------------------------------------------------------------------------------#

#Hour Count = 87 :<
# Solver was so easy...... why is UI so hard..... kms

#To Do for next time possible>
#Wires are still frying me, make it so that there is a jut outward from the terminal before the A* kicks in
#Moving Components must update all Wires attached
#Components colliding with wires must be updated
#Component Body must never be traversable
#Rotation needs severe fixing
#Preview Wire probably needs to be scrapped or....


class Scene():
    def __init__(self):
        self.Components = []
        self.Connections = []

        self.Nodes = []
        self.Circuits = []
        
        self.Solver = Solver()
        self.TimeStep = 0.01
        self.SimulationTime = 0.0
        
    def addComponent(self, ComponentInstance):
        self.Components.append(ComponentInstance)
        ComponentInstance.Scene = self
        self.unresolvedTopology = True
        
    def removeComponent(self, ComponentInstance):
        for TerminalInstance in ComponentInstance.Terminals:
            for ConnectionInstance in list(TerminalInstance.Connections):
                self.removeConnection(TerminalInstance, ConnectionInstance.getRemoteTerminal(TerminalInstance))
                
        self.Components.remove(ComponentInstance)
        ComponentInstance.Scene = None
        self.unresolvedTopology = True
        
    def addConnection(self, TerminalA, TerminalB):
        NewConnection = Connection(TerminalA, TerminalB)
        TerminalA.Connections.append(NewConnection)
        TerminalB.Connections.append(NewConnection)
        self.Connections.append(NewConnection)
        self.unresolvedTopology = True
        
    def removeConnection(self, TerminalA, TerminalB):
        for ConnectionInstance in list(TerminalA.Connections):
            if ConnectionInstance.getRemoteTerminal(TerminalA) is TerminalB:
                self.Connections.remove(ConnectionInstance)
                TerminalA.Connections.remove(ConnectionInstance)
                TerminalB.Connections.remove(ConnectionInstance)
        self.unresolvedTopology = True
    
    def updateScene(self):
        if self.unresolvedTopology == True:
            
            for ComponentInstance in self.Components:
                ComponentInstance.Circuit = None
                for TerminalInstance in ComponentInstance.Terminals:
                    TerminalInstance.Node = None
                    
            self.Nodes = []
            
            for ComponentInstance in self.Components:
                for TerminalInstance in ComponentInstance.Terminals:
                    if TerminalInstance.Node is None:
                        NewNode = Node()
                        NewNode.updateNode(TerminalInstance)
                        self.Nodes.append(NewNode)
                        
            self.Circuits = []
            
            for NodeInstance in self.Nodes:
                if NodeInstance.Circuit is None:
                    NewCircuit = Circuit()
                    NewCircuit.updateCircuit(NodeInstance)
                    self.Circuits.append(NewCircuit)
                    
            self.unresolvedTopology = False
            
    def SimulationStep(self, Steps = 1):
        self.updateScene()
        
        for Step in range(Steps):
            for CircuitInstance in self.Circuits:
                self.Solver.solveCircuit(CircuitInstance, self.TimeStep)
            self.SimulationTime += self.TimeStep
    
class Solver():
    def __init__(self):
        self.Circuit = None
        self.Timestep = None
        
        self.NodeIndexes = {}
        self.BranchIndexes = {}
        self.Matrix = None
        self.Vector = None
        self.Solution = None
        
    def solveCircuit(self, CircuitInstance, TimeStep):
        self.Circuit = CircuitInstance
        self.TimeStep = TimeStep
        
        self.NodeIndexes = {}
        
        for NodeInstance in CircuitInstance.Nodes:
            if NodeInstance is not CircuitInstance.Ground:
                self.NodeIndexes[NodeInstance] = len(self.NodeIndexes)
                
        self.BranchIndexes = {}
        Size = len(self.NodeIndexes)
        
        for ComponentInstance in CircuitInstance.Components:
            if ComponentInstance.BranchCount > 0:
                self.BranchIndexes[ComponentInstance] = Size
                Size += ComponentInstance.BranchCount
                
        self.Matrix = zeros(Size, Size)
        self.Vector = zeros(Size)
        
        for ComponentInstance in CircuitInstance.Components:
            ComponentInstance.Stamp(self)
            
        try:
            if Size > 0:
                self.Solution = linalg.solve(self.Matrix, self.Vector)
            else:
                self.Solution = zeros(0)
        except linalg.LinAlgError:
            CircuitInstance.Error = "Circuit cannot be solved (shorted or parallel voltage sources, or a floating part)"
            return False
        
        for NodeInstance, Index in self.NodeIndexes.items():
            NodeInstance.Voltage = float(self.Solution[Index])
            
        CircuitInstance.Ground.Voltage = 0.0
        
        for ComponentInstance in CircuitInstance.Components:
            ComponentInstance.updateComponent(self)
            
        CircuitInstance.Error = None
        return True
    
class Circuit():
    def __init__(self):
        self.Nodes = []
        self.Components = []
        self.Ground = None
        
    def updateCircuit(self, StartingNode):
        self.Nodes = []
        self.Components = []
        self.Ground = StartingNode
        
        VisitedNodes = set()
        VisitedComponents = set()
        
        UnvisitedNodes = [StartingNode]
        
        while UnvisitedNodes != []:
            CurrentNode = UnvisitedNodes.pop()
            
            if CurrentNode not in VisitedNodes:
                
                VisitedNodes.add(CurrentNode)
                CurrentNode.Circuit = self
                self.Nodes.append(CurrentNode)
                
                for TerminalInstance in CurrentNode.Terminals:
                    ComponentInstance = TerminalInstance.Component
                    
                    if ComponentInstance not in VisitedComponents:
                        
                        VisitedComponents.add(ComponentInstance)
                        ComponentInstance.Circuit = self
                        self.Components.append(ComponentInstance)
                        
                        for CounterpartTerminal in ComponentInstance.Terminals:
                            UnvisitedNodes.append(CounterpartTerminal.Node)
                
class Node():
    def __init__(self):
        self.Circuit = None
        self.Terminals = []
        self.Connections = []
        
        self.Voltage = None
        
    def updateNode(self, StartingTerminal):
        self.Terminals = []
        self.Connections = []
        
        VisitedTerminals = set()
        VisitedConnections = set()
        
        UnvisitedTerminals = [StartingTerminal]
        
        while UnvisitedTerminals != []:
            CurrentTerminal = UnvisitedTerminals.pop()
            
            if CurrentTerminal not in VisitedTerminals:
                
                VisitedTerminals.add(CurrentTerminal)
                CurrentTerminal.Node = self
                self.Terminals.append(CurrentTerminal)
                
                for ConnectionInstance in CurrentTerminal.Connections:
                    if ConnectionInstance not in VisitedConnections:
                        
                        VisitedConnections.add(ConnectionInstance)
                        self.Connections.append(ConnectionInstance)
                        
                    UnvisitedTerminals.append(ConnectionInstance.getRemoteTerminal(CurrentTerminal))
    
class Connection():
    def __init__(self, TerminalA, TerminalB):
        self.TerminalA = TerminalA
        self.TerminalB = TerminalB

    def getRemoteTerminal(self, TerminalInstance):
        if TerminalInstance is self.TerminalA:
            return self.TerminalB
        if TerminalInstance is self.TerminalB:
            return self.TerminalA

class Terminal():
    def __init__(self, ComponentInstance):
        self.Component = ComponentInstance
        self.Connections = []
        self.Node = None
    
class Component():
    def __init__(self):
        self.Terminals = []
        self.Scene = None
        self.Circuit = None
        
"""DemoScene = Scene()

V1 = VoltageSource()
I1 = CurrentSource()
R1 = Resistor()
R2 = Resistor()
R3 = Resistor()
C1 = Capacitor()

V1.setVoltage(12)
I1.setCurrent(0.002)
R1.setResistance(1000)
R2.setResistance(1000)
R3.setResistance(1000)
C1.setCapacitance(0.001)

DemoScene.addComponent(V1)
DemoScene.addComponent(I1)
DemoScene.addComponent(R1)
DemoScene.addComponent(R2)
DemoScene.addComponent(R3)
DemoScene.addComponent(C1)

DemoScene.addConnection(V1.Positive, I1.Negative)

DemoScene.addConnection(I1.Positive, R1.T1)
DemoScene.addConnection(I1.Positive, R2.T1)

DemoScene.addConnection(R1.T2, R3.T1)
DemoScene.addConnection(R2.T2, R3.T1)

DemoScene.addConnection(R3.T2, C1.T1)
DemoScene.addConnection(C1.T2, V1.Negative)

TimeStep = 0.000001 

DemoScene.updateScene()

for Index in range(100000):
    for CircuitInstance in DemoScene.Circuits:
        CircuitInstance.Solve(TimeStep)

for Component in DemoScene.Components:
    print(f"{type(Component).__name__} > {Component.Voltage:.4f}V | {Component.Current:.4f}A | {Component.Power:.4f}W")"""

#------------------------------------------------------------------------------------------------------------------------------#

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        
        self.ComponentBar = ComponentBar()
        self.ComponentPalette = ComponentPalette()
        self.GraphicsView = GraphicsView()
        
        self.ComponentBar.CategoryOpened.connect(self.ComponentPalette.showCategory)
        self.ComponentBar.CategoryClosed.connect(self.ComponentPalette.hide)
        
        self.ComponentPalette.ComponentClicked.connect(self.GraphicsView.addComponentView)
        
        CentralWidget = QWidget()
        Layout = QHBoxLayout(CentralWidget)
        
        Layout.addWidget(self.ComponentBar)
        Layout.addWidget(self.ComponentPalette)
        Layout.addWidget(self.GraphicsView)
        
        self.setCentralWidget(CentralWidget)
        
        self.ToolBar = ToolBar(self)
        self.addToolBar(Qt.ToolBarArea.TopToolBarArea, self.ToolBar)
        
        self.ToolBar.PlayPause.triggered.connect(self.toggleSimulation)
        self.ToolBar.Slower.triggered.connect(lambda: self.ToolBar.setSpeed(self.ToolBar.Speed - 1))
        self.ToolBar.Faster.triggered.connect(lambda: self.ToolBar.setSpeed(self.ToolBar.Speed + 1))
        
        self.SimulationTimer = QTimer(self)
        self.SimulationTimer.setInterval(32) # 60 refeshes per sec
        self.SimulationTimer.timeout.connect(self.SimulationTick)
        
        self.ToolBar.setSpeed(2)
        
        self.StatusBar = StatusBar(self)
        self.setStatusBar(self.StatusBar)
        
    def toggleSimulation(self):
        if self.SimulationTimer.isActive():
            self.SimulationTimer.stop()
            self.ToolBar.PlayPause.setText("> Play")
            self.StatusBar.showMessage("Simulation paused")
        else:
            self.GraphicsView.GraphicsScene.BackendScene.updateScene()
            self.SimulationTimer.start()
            self.ToolBar.PlayPause.setText("|| Pause")
            self.StatusBar.showMessage("Simulation running")
        
    def SimulationTick(self):
        BackendScene = self.GraphicsView.GraphicsScene.BackendScene
        BackendScene.SimulationStep(self.ToolBar.StepsPerTick)
        self.StatusBar.showMessage(f"Simulation running | t = {BackendScene.SimulationTime:.6g} s | speed = {self.ToolBar.SpeedText}")
        
            

class ToolBar(QToolBar):
    def __init__(self, Parent=None):
        super().__init__("Simulation", Parent)

        self.Speed = 2
        self.StepsPerTick = 1
        self.SpeedText = "1x"

        self.PlayPause = QAction("> Play", self)
        self.addAction(self.PlayPause)

        self.addSeparator()

        self.Slower = QAction("− Speed", self)
        self.Faster = QAction("+ Speed", self)
        self.addAction(self.Slower)
        self.addAction(self.Faster)

        self.SpeedLabel = QLabel("  1x")
        self.addWidget(self.SpeedLabel)

    def setSpeed(self, Index):
        self.Speed = max(0, min(5, Index))

        # Solver iterations per ~60 Hz UI refresh.
        Speeds = [1, 2, 4, 8, 16, 32]
        Labels = ["0.25x", "0.5x", "1x", "2x", "4x", "8x"]

        self.StepsPerTick = Speeds[self.Speed]
        self.SpeedText = Labels[self.Speed]
        self.SpeedLabel.setText(f"  {self.SpeedText}")

class StatusBar(QStatusBar):
    pass
       
class ComponentBar(QWidget):
    
    CategoryOpened = Signal(str)
    CategoryClosed = Signal()
    
    def __init__(self):
        super().__init__()
        
        self.setFixedWidth(100)
        
        self.Layout = QVBoxLayout(self)
        
        self.CategoryButtons = []
        
        self.addCategory("Passive")
        self.addCategory("Sources")
        self.addCategory("Semiconductors")
        
    def addCategory(self, Category):
        
        CategoryButton = QPushButton(Category)
        CategoryButton.setCheckable(True)
        
        CategoryButton.clicked.connect(lambda Signal, Button = CategoryButton: self.Clicked(Button, Signal))
        
        self.Layout.addWidget(CategoryButton)
        self.CategoryButtons.append(CategoryButton)
        
    def Clicked(self, Button, Signal):
        
        for OtherButton in self.CategoryButtons:
            
            if OtherButton is not Button:
                OtherButton.setChecked(False)
                
        if Signal == True:
            self.CategoryOpened.emit(Button.text())
        else: 
            self.CategoryClosed.emit()
                                     
class ComponentPalette(QWidget):
    
    ComponentClicked = Signal(str)
    
    def __init__(self):
        super().__init__()
        
        self.Layout = QGridLayout(self)
        
        self.Components = {"Passive" : ["Resistor", "Capacitor", "Inductor", "Transformer"],
                           "Sources" : ["Voltage Source", "Current Source"],
                           "Semiconductors" : ["Diodes", "Transistors", "Thyristors"]}
        
    def showCategory(self, Category):
        
        while self.Layout.count() != 0: #self.Layout.clear()?
            Widget = self.Layout.takeAt(0)
            Widget.widget().deleteLater() 
        
        for Index, Component in enumerate(self.Components[Category]):
            Button = ComponentButton(Component)
            
            Button.ComponentClicked.connect(self.ComponentClicked.emit)
            
            Row = Index // 2
            Column = Index % 2
            
            self.Layout.addWidget(Button, Row, Column)
            
            self.show()
            
class ComponentButton(QPushButton):
    
    ComponentClicked = Signal(str)
    
    def __init__(self, Component):
        super().__init__(Component)
        
        self.Component = Component          
        
        self.clicked.connect(lambda Signal : self.ComponentClicked.emit(self.Component))

        #Drag Button

class GraphicsView(QGraphicsView):
    def __init__(self):
        super().__init__()
        
        self.GraphicsScene = GraphicsScene()
        self.setScene(self.GraphicsScene)
        self.scene().setSceneRect(0, 0, self.viewport().width(), self.viewport().height())
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)
        
    def addComponentView(self, Component):
        
        Position = self.mapToScene(self.viewport().rect().center())
        
        self.GraphicsScene.addComponentItem(Component, Position)
        
    def keyPressEvent(self, Event):
        if Event.key() == Qt.Key.Key_R:
            for Item in self.scene().selectedItems():
                if isinstance(Item, ComponentItem):
                    Item.itemRotate("Clockwise")
            return
                    
        if Event.key() == Qt.Key.Key_Q:
            for Item in self.scene().selectedItems():
                if isinstance(Item, ComponentItem):
                    Item.itemRotate("Anti-Clockwise")
            return
        
        super().keyPressEvent(Event)
      
# Grid snap, wires as extensions of the components
# Electrons starting on every point place of the component      
      
class GraphicsScene(QGraphicsScene):
    def __init__(self):
        super().__init__()
        self.ComponentItems = []
        self.WireItems = []
        self.BackendScene = Scene()
        self.ItemFactories = {"Resistor": lambda: ResistorItem(self),
                              "Voltage Source": lambda: VoltageSourceItem(self),
                              "Current Source": lambda: CurrentSourceItem(self),}
        
        self.CurrentView = False
        self.ElectronView = False
        self.MagneticView = False
        self.TemperatureView = False
        self.SoundView = False
        self.LightView = False
        
        self.GridView = False
        self.GridSize = 20
        
        self.ComponentNameView = False
        self.TerminalNameView = False
        self.NodeNameView = False
        self.CircuitNameView = False
        
        self.WireStartTerminal = None
        self.PreviewWire = None
        self.WireRouter = WireRouter(self)
              
    def addComponentItem(self, Component, Position):
        
        if Component in self.ItemFactories:
        
            ComponentItem = self.ItemFactories[Component]()

            ComponentItem.setPos(Position)
            
            ComponentItem.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIsMovable)
            ComponentItem.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIsSelectable)

            ComponentItem.PoseChanged.connect(self.rerouteWires)

            self.addItem(ComponentItem)  
            self.ComponentItems.append(ComponentItem)
            
            self.BackendScene.addComponent(ComponentItem.Component)
            self.BackendScene.updateScene()
            
    def mousePressEvent(self, Event):
        Item = self.itemAt(Event.scenePos(), QTransform())
        
        if isinstance(Item, TerminalItem):
            self.WireStartTerminal = Item
            
            self.PreviewWire = QGraphicsPathItem()
            self.PreviewWire.setPen(QPen(QColor("#638aa6"), 2, Qt.PenStyle.DashLine))
            self.addItem(self.PreviewWire)
            self.updatePreviewWire(Item.scenePos())
            
            Event.accept()
            return
        
        super().mousePressEvent(Event)
        
    def updatePreviewWire(self, EndPoint):
        HoverItem = self.itemAt(EndPoint, QTransform())
        
        if isinstance(HoverItem, TerminalItem) and HoverItem is not self.WireStartTerminal:
            EndTerminal = HoverItem
        else:
            EndTerminal = None
        
        Points = self.WireRouter.routePath(self.WireStartTerminal, EndPoint, EndTerminal)

        Path = QPainterPath(Points[0])
        for Point in Points[1:]:
            Path.lineTo(Point)

        self.PreviewWire.setPath(Path)
    
    def rerouteWires(self):
        for Wire in self.WireItems:
            if not Wire.ManualRoute:
                Wire.updatePath()
        
    def mouseMoveEvent(self, Event):
        if self.WireStartTerminal is not None:
            self.updatePreviewWire(Event.scenePos())
            
            Event.accept()
            return
        
        super().mouseMoveEvent(Event)
    
    def mouseReleaseEvent(self, Event):
        if self.WireStartTerminal is not None:
            self.removeItem(self.PreviewWire)
            self.PreviewWire = None
            
            Item = self.itemAt(Event.scenePos(), QTransform())
            
            SameComponent = isinstance(Item, TerminalItem) and Item.ComponentItem is self.WireStartTerminal.ComponentItem
            
            if isinstance(Item, TerminalItem) and Item is not self.WireStartTerminal and not SameComponent:
                                
                NewWire = WireItem(self.WireStartTerminal, Item, self)
                
                self.WireStartTerminal.Wires.append(NewWire)
                Item.Wires.append(NewWire)
                
                self.WireItems.append(NewWire)
                self.addItem(NewWire)
                
                self.BackendScene.addConnection(self.WireStartTerminal.Terminal, Item.Terminal)
                self.BackendScene.updateScene()
                
                for ComponentItemInstance in self.ComponentItems:
                    ComponentItemInstance.update()
            
            self.WireStartTerminal = None
            Event.accept()
            return
        
        super().mouseReleaseEvent(Event)
        
class WireItem(QGraphicsPathItem):
    def __init__(self, TerminalA, TerminalB, GraphicsScene):
        super().__init__()
        self.GraphicsScene = GraphicsScene
        self.TerminalA = TerminalA
        self.TerminalB = TerminalB
        
        self.Points = []
        self.ManualRoute = False
        self.DraggingSegment = None
        self.DragStart = None
        
        self.setPen(QPen(QColor("#001733"), 2))
        self.setZValue(-1)
        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIsFocusable)
        self.setAcceptHoverEvents(True)
        
        self.updatePath()
        
    def hoverMoveEvent(self, Event):
        self.setCursor(Qt.CursorShape.SizeAllCursor if self.segmentAt(Event.pos()) is not None else Qt.CursorShape.ArrowCursor)
        
    def segmentAt(self, Position, Tolerance = 6):
        for Index in range(len(self.Points) - 1):
            Line = QLineF(self.Points[Index], self.Points[Index + 1])
            if self.distanceToSegment(Position, Line) < Tolerance:
                return Index
        return None
    
    def distanceToSegment(self, Point, Line):
        LineVector = Line.p2() - Line.p1()
        PointVector = Point - Line.p1()

        LineLengthSquared = QPointF.dotProduct(LineVector, LineVector)
        if LineLengthSquared == 0:
            return QLineF(Point, Line.p1()).length()

        Transposition = QPointF.dotProduct(PointVector, LineVector) / LineLengthSquared
        Transposition = max(0, min(1, Transposition))

        Closest = Line.p1() + LineVector * Transposition
        return QLineF(Point, Closest).length()
    
    def mousePressEvent(self, Event):
            Index = self.segmentAt(Event.pos())
            if Index is not None:
                self.ManualRoute = True
                IsHorizontal = self.Points[Index].y() == self.Points[Index+1].y()
                self.Points.insert(Index + 1, QPointF(self.Points[Index]))
                self.Points.insert(Index + 2, QPointF(self.Points[Index + 1]))
                self.DraggingSegment = (Index + 1, IsHorizontal)
                self.DragStart = Event.pos()
                Event.accept()
                return
            super().mousePressEvent(Event)

    def mouseMoveEvent(self, Event):
        if self.DraggingSegment is not None:
            Index, IsHorizontal = self.DraggingSegment
            Delta = Event.pos() - self.DragStart
            if IsHorizontal:
                self.Points[Index].setY(self.Points[Index].y() + Delta.y())
                self.Points[Index+1].setY(self.Points[Index+1].y() + Delta.y())
            else:
                self.Points[Index].setX(self.Points[Index].x() + Delta.x())
                self.Points[Index+1].setX(self.Points[Index+1].x() + Delta.x())
            self.DragStart = Event.pos()
            self.rebuildPath()
            Event.accept()
            return
        super().mouseMoveEvent(Event)

    def mouseReleaseEvent(self, Event):
        if self.DraggingSegment is not None:
            for Point in self.Points:
                Point.setX(round(Point.x() / 5) * 5)
                Point.setY(round(Point.y() / 5) * 5)
            self.DraggingSegment = None
            self.rebuildPath()
        super().mouseReleaseEvent(Event)

    def rebuildPath(self):
        Path = QPainterPath(self.Points[0])
        for Point in self.Points[1:]:
            Path.lineTo(Point)
        self.setPath(Path)
    
    def updatePath(self):
        if self.ManualRoute:
            self.Points[0] = self.TerminalA.scenePos()
            self.Points[-1] = self.TerminalB.scenePos()
        else:
            self.Points = self.GraphicsScene.WireRouter.routePath(self.TerminalA, self.TerminalA.scenePos(), self.TerminalB)

        self.rebuildPath()
               
class WireRouter(): # ~A*~ :>
    def __init__(self, GraphicsScene):
        self.GraphicsScene = GraphicsScene
        self.Directions = ((1, 0), (-1, 0), (0, 1), (0, -1))
        self.SearchMargin = 8
        self.BridgeDistance = 40
        
    def buildObstacles(self): # Remove ParentCOmponents (figure out how to do so wtht breaking everything :<) ########URGENT########
        Obstacles = set()
    
        for Item in self.GraphicsScene.ComponentItems:

            Rect = Item.sceneBoundingRect()
            MinX = int(Rect.left() // self.GraphicsScene.GridSize)
            MaxX = int((Rect.right() - 1e-9) // self.GraphicsScene.GridSize)
            MinY = int(Rect.top() // self.GraphicsScene.GridSize)
            MaxY = int((Rect.bottom() - 1e-9) // self.GraphicsScene.GridSize)

            for X in range(MinX, MaxX + 1):
                for Y in range(MinY, MaxY + 1):
                    Obstacles.add((X, Y))
                        
        return Obstacles
    
    def buildBridges(self, StartTerminal, EndPoint, EndTerminal):
        StartPoint = StartTerminal.scenePos()
        StartBridge = StartTerminal.getBridgePoint(self.BridgeDistance)
        
        if EndTerminal is not None:
            EndPoint = EndTerminal.scenePos()
            EndBridge = EndTerminal.getBridgePoint(self.BridgeDistance)
        else:
            EndBridge = EndPoint
            
        return (StartBridge, EndBridge, StartPoint, EndPoint)
    
    def routePath(self, StartTerminal, MousePoint, EndTerminal = None):
        
        StartBridge, EndBridge, StartPoint, EndPoint = self.buildBridges(StartTerminal, MousePoint, EndTerminal)
        
        Start = (round(StartBridge.x() / self.GraphicsScene.GridSize), round(StartBridge.y() / self.GraphicsScene.GridSize))
        End = (round(EndBridge.x() / self.GraphicsScene.GridSize), round(EndBridge.y() / self.GraphicsScene.GridSize))
        
        if Start == End:
            GridPath = [StartBridge, EndBridge]
        else:
            Obstacles = self.buildObstacles()
            Obstacles.discard(Start)
            Obstacles.discard(End)
            
            MinX = min(Start[0], End[0]) - self.SearchMargin
            MaxX = max(Start[0], End[0]) + self.SearchMargin
            MinY = min(Start[1], End[1]) - self.SearchMargin
            MaxY = max(Start[1], End[1]) + self.SearchMargin
            
            SearchCells = (MaxX - MinX + 1) * (MaxY - MinY + 1)
            
            #StateLimit = min(self.MaxStates, max(5000, SearchCells * 4))
            StartState = (Start, None)
            
            OpenSet = [(0, 0, StartState)]
            CostSoFar = {StartState: 0}
            CameFrom = {}
            Counter = 0
            #Explored = 0
            
            GridPath = None
        
            while OpenSet != []:
                _, _, State = heapq.heappop(OpenSet)
                Current, Incoming = State

                if Current == End:
                    GridPath = [State]
                    while GridPath[-1] != StartState:
                        GridPath.append(CameFrom[GridPath[-1]])
                    GridPath.reverse()
                    break

                for Direction in self.Directions:
                    Next = (Current[0] + Direction[0], Current[1] + Direction[1])
                    if (MinX <= Next[0] <= MaxX and MinY <= Next[1] <= MaxY) and Next not in Obstacles:
                        
                        if Incoming in (None, Direction):
                            TurnCost = 0
                        else:
                            TurnCost = 5

                        NewCost = CostSoFar[State] + 1 + TurnCost
                        NextState = (Next, Direction)

                        if NewCost < CostSoFar.get(NextState, float('inf')):
                            CostSoFar[NextState] = NewCost
                            CameFrom[NextState] = State
                            Heuristic = abs(End[0] - Next[0]) + abs(End[1] - Next[1])
                            Counter += 1
                            heapq.heappush(OpenSet, (NewCost + Heuristic, Counter, NextState))
                            
            if GridPath is None:
                GridPath = [StartBridge, EndBridge]
                
        if GridPath and isinstance(GridPath[0], tuple):
            ScenePath = []
            for Point, _ in GridPath:
                ScenePath.append(QPointF(Point[0] * self.GraphicsScene.GridSize, Point[1] * self.GraphicsScene.GridSize))
        else:
            ScenePath = GridPath
                
        FullPath = [StartPoint] + ScenePath + [EndPoint]
        return self.compressPath(FullPath)
    
    #AAAAAAAAAAAAAAAAAAAAHHHHHHHHHHHHHHHHHHHHHH
    #Find points based on directions - directions chosen based on heuristic, Orthogonal pattern, we remove obstacles.   
                
    #Fixed so happy with self :)
    # Next time - fix assymetric bridges
    # Design a way so that components can not interact with bounding rect, same with wires
    # (tailor bounding rects more closely to components)
    # Add a simulation button, allowing time to be sped up, slown down, paused and played in taskbar
    # Add a view section on the taskbar for electrons and stuff.
    # Overhaul context menu + Name, Value, Graph (V,I, / T graph)
                
                
                
    def compressPath(self, ScenePath): #Turns many points into one line.
        if len(ScenePath) < 3:
            return ScenePath
        
        CompressedPath = [ScenePath[0]]
        
        for Index in range(1, len(ScenePath) - 1):
            Previous, Current, Next = ScenePath[Index - 1], ScenePath[Index], ScenePath[Index + 1]
            SameLine = (Previous.y() == Current.y() == Next.y()) or (Previous.x() == Current.x() == Next.x())
            if not SameLine:
                CompressedPath.append(Current)
        CompressedPath.append(ScenePath[-1])
        return CompressedPath
        
    #Find Valid points
    # Route via those valid points
    
class TerminalItem(QGraphicsObject):
    def __init__(self, ComponentItem, Terminal, LocalX, LocalY):
        super().__init__(parent = ComponentItem)
        
        self.ComponentItem = ComponentItem
        self.Terminal = Terminal
        self.Wires = []
        self.LocalOffset = QPointF(LocalX, LocalY)
        
        self.setZValue(1)
        self.setAcceptedMouseButtons(Qt.MouseButton.LeftButton)
        self.setPos(LocalX, LocalY)
        
    def boundingRect(self):
        return QRectF(-4, -4, 8, 8)
    
    def paint(self, Painter, Option, Widget = None):
        Painter.setBrush(QBrush(QColor("#294e73")))
        Painter.setPen(Qt.PenStyle.NoPen)
        Painter.drawEllipse(self.boundingRect())
        
    def getBridgePoint(self, Distance = 20):
        if abs(self.LocalOffset.x()) >= abs(self.LocalOffset.y()):
            if self.LocalOffset.x() > 0:
                Direction = QPointF(1, 0)
            else:
                Direction = QPointF(-1, 0)
        else:
            if self.LocalOffset.y() > 0:
                Direction = QPointF(0, 1)
            else:
                Direction = QPointF(0, -1)
        
        return self.scenePos() + QTransform().rotate(self.ComponentItem.rotation()).map(Direction) * Distance
        
class ComponentItem(QGraphicsObject):     
    #Symbol | Name (V) | Value (V) | Rotation (V) | Position (V) | Component | TerminalItems
    
    PoseChanged = Signal()
    DisplayChanged = Signal()
       
    def __init__(self, GraphicsScene):
        super().__init__()
        self.GraphicsScene = GraphicsScene
        self.Component = None
        self.TerminalItems = []
        
        self.Value = None
        self.Name = None#f"{self.Root}{self.GraphicsScene.ItemAmounts[type(self).__name__]}"
        self.Renderer = None #QSvgRenderer(self.SvgPath)

        self.Moving = False
        self.RotationStage = 0 # 0-7 | 45 degrees
        
        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIsMovable)
        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIsSelectable)
        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemSendsGeometryChanges)
        
        self.setZValue(0)
        self.setAcceptHoverEvents(True)
        self.setPos(0,0)
        
        self.PoseChanged.connect(self.updateWires)
        
    def updateWires(self):
        for Terminal in self.TerminalItems:
            for Wire in Terminal.Wires:
                Wire.updatePath()
        
    def generateName(self, Root):
        Number = 1
        Names = []
        
        for Item in self.GraphicsScene.ComponentItems:
            Names.append(Item.Name)
        
        while f"{Root}{Number}" in Names:
            Number += 1
        return f"{Root}{Number}"
    
    def changeName(self, NewName):
        Names = []
        
        for Item in self.GraphicsScene.ComponentItems:
            Names.append(Item.Name)
            
        if NewName not in Names:
            self.Name = NewName
            self.DisplayChanged.emit()
    
    def boundingRect(self): #TEXT Auto function... try change name = break
        return QRectF(-70, -60, 140, 120)
    
    def paint(self, Painter, Option, Widget = None):
        
        #Same Here ^^^^
        Painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        Painter.setPen(QPen(Qt.GlobalColor.darkGray, 1)) #COLOUR, THICNESS
        Painter.setFont(QFont("Sans", 9))
        Painter.drawText(QRectF(-65, 40, 130, 20), Qt.AlignmentFlag.AlignCenter, self.Name)
                
        self.Renderer.render(Painter, QRectF(-60, -30, 120, 60))
        
    def itemChange(self, Change, NewPosition): #QT Called Function
        if Change == QGraphicsItem.GraphicsItemChange.ItemPositionChange:
            return QPointF(round(NewPosition.x() / 20) * 20, round(NewPosition.y() / 20) * 20) #GRID Snapping.
        if Change == QGraphicsItem.GraphicsItemChange.ItemPositionHasChanged:
            self.PoseChanged.emit()
            
        return super().itemChange(Change, NewPosition)
        
    def itemRotate(self, Direction):
        Directions = {"Clockwise" : 1, "Anti-Clockwise" : -1}
        self.RotationStage = (self.RotationStage + Directions[Direction]) % 8
        self.setRotation(self.RotationStage * 45)
        self.PoseChanged.emit()
               
        #Position changed -> update view topology (WIRES)
        
    def buildContextMenu(self):
        
        Menu = QMenu()
        
        NameAction = Menu.addAction(f"Name: {self.Name}")
        ValueAction = Menu.addAction(f"Value: {self.Value}")
        Menu.addSeparator()
        CurrentAction = Menu.addAction(f"Current: {self.Component.Current:.4f}")
        VoltageAction = Menu.addAction(f"Voltage: {self.Component.Voltage:.4f}")
        PowerAction = Menu.addAction(f"Power: {self.Component.Power:.4f}")
        
        CurrentAction.setEnabled(False)
        VoltageAction.setEnabled(False)
        PowerAction.setEnabled(False)
        
        return Menu, NameAction, ValueAction
    
    def editName(self): #Transition to QLabel?
        Views = self.GraphicsScene.views()
        if Views != []:
            Parent = Views[0]
        else:
            Parent = None
        
        NewName, Flag = QInputDialog.getText(Parent, "Edit Name", "Name:", text = self.Name)
        if Flag and NewName:
            self.changeName(NewName)
    
    def editValue(self):
        Views = self.GraphicsScene.views()
        if Views != 0:
            Parent = Views[0]
        else:
            None
        
        NewValue, Flag = QInputDialog.getDouble(Parent, "Edit Value", "Value:", value = self.getValue()) #LOOK INTO THIS MORE 
        if Flag == True:
            self.setValue(NewValue)
            
    def contextMenuEvent(self, Event):
        Menu, NameAction, ValueAction = self.buildContextMenu()
        
        Action = Menu.exec(Event.screenPos())
        
        if Action == NameAction:
            self.editName()
        elif Action == ValueAction:
            self.editValue()        
        
    def Serialise(self):
        return {"Type" : type(self.Component).__name__,
                "Name" : self.Name,
                "Value" : self.Value,
                "Position" : [self.pos().x(), self.pos().y()],
                "Rotation" : self.RotationStage}
        
        #########self.Terminals = None
          
    # Display it's Name
    # Display it's Values
    # Know It's Components
    # Know It's position
    # Know it's Terminal Positions
    # Know It's Appearence

class VoltageSourceItem(ComponentItem):
    def __init__(self, GraphicsScene):
        super().__init__(GraphicsScene)
        
        self.Component = VoltageSource()
        self.Component.setVoltage(12) #default
        self.TerminalItems = [TerminalItem(self, self.Component.Positive, 0, -60, ), TerminalItem(self, self.Component.Negative, 0, 60 )]
        
        self.Name = self.generateName("V")
        self.Renderer = QSvgRenderer("Symbols/VoltageSource.svg")
        self.Value = self.getValue()
  
    def getValue(self):
        return self.Component.Voltage
    
    def setValue(self, Value):
        self.Component.setVoltage(Value)
        self.Value = Value 
        self.DisplayChanged.emit() 
        
    # def getName(self):
    # def setName(self):
    
class CurrentSourceItem(ComponentItem):
    def __init__(self, GraphicsScene):
        super().__init__(GraphicsScene)
        
        self.Component = CurrentSource()
        self.Component.setCurrent(0.002)
        self.TerminalItems = [TerminalItem(self, self.Component.Positive, 0, -60, ), TerminalItem(self, self.Component.Negative, 0, 60)]
        
        self.Name = self.generateName("I")
        self.Renderer = QSvgRenderer("Symbols/CurrentSource.svg")
        self.Value = self.getValue()
        
    def getValue(self):
        return self.Component.Current
    
    def setValue(self, Value):
        self.Component.setCurrent(Value)
        self.Value = Value
        self.DisplayChanged.emit() 

class ResistorItem(ComponentItem):
    def __init__(self, GraphicsScene):
        super().__init__(GraphicsScene)
        
        self.Component = Resistor()
        self.Component.setResistance(1000)
        self.TerminalItems = [TerminalItem(self, self.Component.T1, -40, 0), TerminalItem(self, self.Component.T2, 40, 0,)]
        
        self.Name = self.generateName("R")
        self.Renderer = QSvgRenderer("Symbols/Resistor.svg")
        self.Value = self.getValue()
        
    def getValue(self):
        return self.Component.Resistance
    
    def setValue(self, Value):
        self.Component.setResistance(Value)
        self.Value = Value
        self.DisplayChanged.emit() 
        
class CapacitorItem(ComponentItem):
    def __init__(self, GraphicsScene):
        super().__init__(GraphicsScene)
        
        self.Component = Capacitor()
        self.Component.setCapacitance(0.001)
        self.TerminalItems = [TerminalItem(self, self.Component.T1, -40, 0), TerminalItem(self, self.Component.T2, 40, 0,)]
        
        self.Name = self.generateName("C")
        self.Renderer = QSvgRenderer("Symbols/Capacitor.svg")
        self.Value = self.getValue()
        
    def getValue(self):
        return self.Component.Capacitance
    
    def setValue(self, Value):
        self.Component.setCapacitance(Value)
        self.Value = Value
        self.DisplayChanged.emit()

class InductorItem(ComponentItem):
    def __init__(self, GraphicsScene):
        super().__init__(GraphicsScene)
        
        self.Component = Inductor()
        self.Component.setInductance(0.001)
        self.TerminalItems = [TerminalItem(self, self.Component.T1, -40, 0), TerminalItem(self, self.Component.T2, 40, 0,)]
        
        self.Name = self.generateName("L")
        self.Renderer = QSvgRenderer("Symbols/Inductor.svg")
        self.Value = self.getValue()
        
    def getValue(self):
        return self.Component.Inductance
    
    def setValue(self, Value):
        self.Component.setInductance(Value)
        self.Value = Value
        self.DisplayChanged.emit() 

Application = QApplication(argv)
Window = MainWindow()
Window.show()
Application.exec()

#CHECKLIST
#GUI
#ComponentBar
# -> Drag & drop components
#GraphicsScene
#Other UI -> Clear button
#Scene functions, deleting components, copying, pasting, connecting terminals [WIRES]
#Simulate and calculate circuits
#Save and load circuits
#
#TO FIX
#floating circuits [y]
#singular matrices [y]
#disconnected components
#duplicate connections
#zero-resistance resistors
#voltage-source loops
#current-source cutsets
#circuits without voltage sources
#circuits with no resistive path
#explicit ground

#GUI EXPLICIT FEATURES

#Class Topology - Main Window : Component Bar, Component Palette, ComponentButtons, Graphics View, Graphics Scene, ComponentItem, Resistor Item....., Toolbar, StatusBar
#Component Palette and bar
#Status and Tool Bar.
#Draw Components
#Drag and drop components from a component palette onto a scene
#Ability to edit component's variable, name and rotation
#Connect component's to eachother using wire items
#Live updating
#Snap to grid feature
#Commands:
# -Undo & Redo
# -Copy, Paste & Cut
# -Select (Single and Area)
# -Delete & Add
# -Clear
# -Save & Load
# -Rotate 360/8 for each component
#Views - Togglable views to see Heat, Sound, Light, Magnetic Field lines, Electron flow, Current Flow aswell as grid. Outputted from components (May also need to come with a size variable for heat? or just fixed dissipation variables for each)


#TOFIX

#SVG files incorrect pathing?
#Wires need to be dynamic
#Crashes every time after drop event
#ADD SELECT AREA
#Wires don't move with components

#When wires overlap - Curve a bit, signifying it
# Wires should not be able to go through components
# Should be able to select and move components

#SIMULATE
#Textures
#Aesthetics
#

#New plan
#Have an in built grid on to of the Graphics Scene
#Have wires and components connected, scaling them
#Anything on a grid square where a wire is, becomes connected to that component.
#Have electrons on this grid.
#
#
#
#
#
#
