import sys
import json
import heapq

from PySide6.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QGraphicsView, QGraphicsScene
from PySide6.QtCore import Qt, QObject, Signal

from numpy import linalg, zeros
from collections import deque

class Scene():
    def __init__(self):
        self.Components = []
        self.Connections = []

        self.Nodes = []
        self.Circuits = []
        
        self.Solver = Solver()
        self.TimeStep = 0.01
        self.SimulationTime = 0.0
        
        self.UnresolvedTopology = True
        self.Errors = []
        
    def addComponent(self, ComponentInstance):
        self.Components.append(ComponentInstance)
        ComponentInstance.Scene = self
        self.unresolvedTopology = True
        return ComponentInstance
        
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
            self.Errors = []
            for CircuitInstance in self.Circuits:
                if not self.Solver.solveCircuit(CircuitInstance, self.TimeStep):
                    self.Errors.append(CircuitInstance.Error)
                    
            if self.Errors != []:
                return False
                
            self.SimulationTime += self.TimeStep
            
        return True
    
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
                
        self.Matrix = zeros((Size, Size))
        self.Vector = zeros(Size)
        
        for ComponentInstance in CircuitInstance.Components:
            ComponentInstance.stampComponent(self)
            
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
    
    def getNodeIndex(self, TerminalInstance):
        if TerminalInstance.Node is self.Circuit.Ground:
            return None
        return self.NodeIndexes[TerminalInstance.Node]
    
    def getBranchIndex(self, ComponentInstance, Number = 0):
        return self.BranchIndexes[ComponentInstance] + Number
    
    def getVoltage(self, TerminalInstance):
        Index = self.getNodeIndex(TerminalInstance)
        if Index is None:
            return 0.0
        else:
            return float(self.Solution[Index])

    def getBranchCurrent(self, ComponentInstance, Number = 0):
        return float(self.Solution[self.getBranchIndex(ComponentInstance, Number)])
    
    def addMatrix(self, Row, Column, Value):
        if Row is not None and Column is not None:
            self.Matrix[Row][Column] += Value
        
    def addVector(self, Row, Value):
        if Row is not None:
            self.Vector[Row] += Value
            
    def stampBranch(self, NodeIndexA, NodeIndexB, Branch):
        self.addMatrix(NodeIndexA, Branch, 1)
        self.addMatrix(NodeIndexB, Branch, -1)
        self.addMatrix(Branch, NodeIndexA, 1)
        self.addMatrix(Branch, NodeIndexB, -1)
        
    def stampConductance(self, NodeIndexA, NodeIndexB, Value):
        self.addMatrix(NodeIndexA, NodeIndexA, Value)
        self.addMatrix(NodeIndexB, NodeIndexB, Value)
        self.addMatrix(NodeIndexA, NodeIndexB, -Value)
        self.addMatrix(NodeIndexB, NodeIndexA, -Value)
        
    def stampCurrent(self, NodeIndexA, NodeIndexB, Value):
        self.addVector(NodeIndexA, Value)
        self.addVector(NodeIndexB, -Value)
    
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
    BranchCount = 0
    def __init__(self):
        self.Terminals = []
        self.Scene = None
        self.Circuit = None
        
        self.Voltage = None
        self.Current = None
        self.Power = None
        
    def addTerminal(self):
        NewTerminal = Terminal(self)
        self.Terminals.append(NewTerminal)
        return NewTerminal
    
    def stampComponent(self, Solver):
        pass
    
    def updateComponent(self, Solver):
        pass
    
    def setResult(self, Voltage, Current):
        self.Voltage = Voltage
        self.Current = Current
        self.Power = Voltage * Current

class VoltageSource(Component):
    BranchCount = 1
    def __init__(self):
        super().__init__()
        
        self.Positive = self.addTerminal()
        self.Negative = self.addTerminal()
       
        self.SourceVoltage = None 
        
    def setVoltage(self, Voltage):
        self.SourceVoltage = Voltage
        
    def stampComponent(self, Solver):
        Branch = Solver.getBranchIndex(self)
        
        Solver.stampBranch(Solver.getNodeIndex(self.Positive), Solver.getNodeIndex(self.Negative), Branch)
        Solver.addVector(Branch, self.SourceVoltage)
        
    def updateComponent(self, Solver):
        Voltage = Solver.getVoltage(self.Positive) - Solver.getVoltage(self.Negative)
        Current = Solver.getBranchCurrent(self)
        
        self.setResult(Voltage, Current)

class CurrentSource(Component):
    def __init__(self):
        super().__init__()
        
        self.Positive = self.addTerminal()
        self.Negative = self.addTerminal()
        
        self.SourceCurrent = None
        
    def setCurrent(self, Current):
        self.SourceCurrent = Current
        
    def stampComponent(self, Solver):
        Solver.stampCurrent(Solver.getNodeIndex(self.Positive), Solver.getNodeIndex(self.Negative), self.SourceCurrent)
        
    def updateComponent(self, Solver):
        Voltage = Solver.getVoltage(self.Positive) - Solver.getVoltage(self.Negative)
        Current = -self.SourceCurrent
        
        self.setResult(Voltage, Current)

class Resistor(Component):
    def __init__(self):
        super().__init__()
        
        self.T1 = self.addTerminal()
        self.T2 = self.addTerminal()
        
        self.Resistance = None
        
    def setResistance(self, Resistance):
        self.Resistance = Resistance
        
    def stampComponent(self, Solver):
        Conductance = 1 / self.Resistance
        
        Solver.stampConductance(Solver.getNodeIndex(self.T1), Solver.getNodeIndex(self.T2), Conductance)
        
    def updateComponent(self, Solver):
        Voltage = Solver.getVoltage(self.T1) - Solver.getVoltage(self.T2)
        Current = Voltage / self.Resistance
        
        self.setResult(Voltage, Current)
        
class Capacitor(Component):
    def __init__(self):
        super().__init__()
        
        self.T1 = self.addTerminal()
        self.T2 = self.addTerminal()
        
        self.Capacitance = None
        
        self.PreviousVoltage = 0.0
        
    def setCapacitance(self, Capacitance):
        self.Capacitance = Capacitance

    def stampComponent(self, Solver):
        Conductance = self.Capacitance / Solver.TimeStep
        Equivalent = Conductance * self.PreviousVoltage
        
        Solver.stampConductance(Solver.getNodeIndex(self.T1), Solver.getNodeIndex(self.T2), Conductance)
        Solver.stampCurrent(Solver.getNodeIndex(self.T1), Solver.getNodeIndex(self.T2), Equivalent)
        
    def updateComponent(self, Solver):
        Voltage = Solver.getVoltage(self.T1) - Solver.getVoltage(self.T2)
        Conductance = self.Capacitance / Solver.TimeStep
        Equivalent = Conductance * self.PreviousVoltage
        Current = Conductance * Voltage - Equivalent
 
        self.setResult(Voltage, Current)
        self.PreviousVoltage = Voltage
        
class Inductor(Component):
    BranchCount = 1
    def __init__(self):
        super().__init__()
        
        self.T1 = self.addTerminal()
        self.T2 = self.addTerminal()
        
        self.Inductance = None
        
        self.PreviousCurrent = 0.0
        
    def setInductance(self, Inductance):
        self.Inductance = Inductance
        
    def stampComponent(self, Solver):
        Branch = Solver.getBranchIndex(self)
        Resistance = self.Inductance / Solver.TimeStep
        
        Solver.stampBranch(Solver.getNodeIndex(self.T1), Solver.getNodeIndex(self.T2), Branch)
        Solver.addMatrix(Branch, Branch, -Resistance)
        Solver.addVector(Branch, -Resistance * self.PreviousCurrent)
        
    def updateComponent(self, Solver):
        Voltage = Solver.getVoltage(self.T1) - Solver.getVoltage(self.T2)
        Current = Solver.getBranchCurrent(self)
        
        self.setResult(Voltage, Current)
        self.PreviousCurrent = Current
    
#Backend scene tickets
#Ground (need to implement a one-pin component instead of selecting first node)
#Need to validate parameters
#Adding semiconductors
#Multi-terminal components } Transistors/semiconductors
#Dynami component clamping

#-----------------------------------------------------------------------------------------------------------------------------------------#

class ComponentAdapter(QObject):
    
    ValueChanged = Signal()
    Updated = Signal()

    def __init__(self, ComponentInstance):
        super().__init__()
        
        self.Component = ComponentInstance
        self.History = deque()
        
        self.HistoryDuration = 60.0
                
    def RecordStep(self, Time):
        self.History.append((Time, self.Component.Voltage, self.Component.Current))

        while self.History and (Time - self.History[0][0]) > self.HistoryDuration:
            self.History.popleft()

        self.Updated.emit()
        
class VoltageSourceAdapter(ComponentAdapter):
    def setVoltage(self, Voltage):
        self.Component.setVoltage(Voltage)
        self.ValueChanged.emit()
 
class CurrentSourceAdapter(ComponentAdapter):
    def setCurrent(self, Current):
        self.Component.setCurrent(Current)
        self.ValueChanged.emit()        

class ResistorAdapter(ComponentAdapter):
    def setResistance(self, Resistance):
        self.Component.setResistance(Resistance)
        self.ValueChanged.emit()
 
class CapacitorAdapter(ComponentAdapter):
    def setCapacitance(self, Capacitance):
        self.Component.setCapacitance(Capacitance)
        self.ValueChanged.emit()
 
class InductorAdapter(ComponentAdapter):
    def setInductance(self, Inductance):
        self.Component.setInductance(Inductance)
        self.ValueChanged.emit()
   
class SceneAdapter(QObject):
    
    ComponentAdded = Signal()
    ComponentRemoved = Signal()
    AcceptedStep = Signal()
    ErrorRaised = Signal()
    
    ComponentTypes = {"VoltageSource": (VoltageSource, VoltageSourceAdapter),
                      "CurrentSource": (CurrentSource, CurrentSourceAdapter),
                      "Resistor":      (Resistor,      ResistorAdapter),
                      "Capacitor":     (Capacitor,     CapacitorAdapter),
                      "Inductor":      (Inductor,      InductorAdapter), }    
    
    def __init__(self):
        super().__init__()
        
        self.Scene = Scene()
        self.ComponentAdapters = []
    
    def createComponent(self, TypeName):
        BackendClass, AdapterClass = self.ComponentTypes[TypeName]
        
        NewComponent = BackendClass()
        self.Scene.addComponent(NewComponent)
        
        NewAdapter = AdapterClass(NewComponent, self)
        self.ComponentAdapters.append(NewAdapter)
        
        self.ComponentAdded.emit(NewAdapter)
        return NewAdapter
    
    def deleteComponent(self, ComponentAdapterInstance):
        self.Scene.removeComponent(ComponentAdapterInstance.Component)
        self.ComponentAdapters.remove(ComponentAdapterInstance)
        
        self.ComponentRemoved.emit(ComponentAdapterInstance)
        ComponentAdapterInstance.setParent(None)
    
    def createConnection(self, TerminalA, TerminalB):
        self.Scene.addConnection(TerminalA, TerminalB)
    
    def deleteConnection(self, TerminalA, TerminalB):
        self.Scene.addConnection(TerminalA, TerminalB)
    
    def SimulationStep(self, Steps = 1):
        for Step in range(Steps):
            if not self.Scene.SimulationStep(1):
                self.ErrorRaised.emit(list(self.Scene.Errors))
                return False

            for ComponentAdapterInstance in self.ComponentAdapters:
                ComponentAdapterInstance.RecordStep(self.Scene.SimulationTime)
                
            self.AcceptedStep.emit(self.Scene.SimulationTime)
 
#-----------------------------------------------------------------------------------------------------------------------------------------#

class CategoryBar(QWidget):
    def __init__(self):
        super().__init__()
 
        self.setStyleSheet("background-color: #001C40; color: #79ccff; margin: 5px")
 
        Layout = QVBoxLayout(self)
        Layout.setContentsMargins(0, 0, 0, 0)
        Layout.addWidget(QLabel("Category Bar"))    
        
class ComponentBar(QWidget):
    def __init__(self):
        super().__init__()
        
        self.setStyleSheet("background-color: #001C40; color: #79ccff; margin: 5px")
 
        Layout = QVBoxLayout(self)
        Layout.setContentsMargins(0, 0, 0, 0)
        Layout.addWidget(QLabel("Component Bar"))        

class OscilloscopePanel(QWidget):
    def __init__(self):
        super().__init__()
    
        self.setStyleSheet("background-color: #001C40; color: #79ccff; margin: 5px")
        
        Layout = QVBoxLayout(self)
        Layout.setContentsMargins(0, 0, 0, 0)
        Layout.addWidget(QLabel("Oscilloscope"))

class SimulationPanel(QWidget):
    def __init__(self):
        super().__init__()
        
        self.setStyleSheet("background-color: #001C40; color: #79ccff; margin: 5px")
        
        Layout = QVBoxLayout(self)
        Layout.setContentsMargins(0, 0, 0, 0)
        Layout.addWidget(QLabel("SimulationPanel"))
    
class ToolBar(QWidget):
    def __init__(self):
        super().__init__()
        
        self.setStyleSheet("background-color: #001C40; color: #79ccff; margin: 5px")
        
        Layout = QVBoxLayout(self)
        Layout.setContentsMargins(0, 0, 0, 0)
        Layout.addWidget(QLabel("ToolBar"))
        
class StatusBar(QWidget):
    def __init__(self):
        super().__init__()
        
        self.setStyleSheet("background-color: #001C40; color: #79ccff; margin: 5px")
        
        Layout = QVBoxLayout(self)
        Layout.setContentsMargins(0, 0, 0, 0)
        Layout.addWidget(QLabel("StatusBar"))
    
class CanvasView(QGraphicsView):
    def __init__(self):
        super().__init__()
        
        self.setStyleSheet("background-color: #002451; color: #79ccff; margin: 5px")
        
        self.CanvasScene = QGraphicsScene(self)
        self.setScene(self.CanvasScene)
        
#------------------------------------------------------------------------------------------------------------------------------------------#

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        
        self.setWindowTitle("SPICE Simulator")
        self.resize(1400, 900)
        
        #-------------------------------------------------#
        
        self.ToolBar = ToolBar()
        
        self.StatusBar = StatusBar()
        
        #-------------------------------------------------#
        
        self.CategoryBar = CategoryBar()
        self.ComponentBar = ComponentBar()
        
        PalettePanel = QWidget()
        PalettePanelLayout = QHBoxLayout(PalettePanel)
        PalettePanelLayout.setContentsMargins(0, 0, 0, 0)
        PalettePanelLayout.setSpacing(0)
 
        PalettePanelLayout.addWidget(self.CategoryBar, 1)
        PalettePanelLayout.addWidget(self.ComponentBar, 2)        
        
        #--------------------------------------------------#
        
        self.CanvasView = CanvasView()
        
        CanvasPanel = QWidget()
        CanvasPanelLayout = QVBoxLayout(CanvasPanel)
        CanvasPanelLayout.setContentsMargins(0, 0, 0, 0)
        CanvasPanelLayout.setSpacing(0)
        
        CanvasPanelLayout.addWidget(self.CanvasView)
        
        #--------------------------------------------------#
        
        self.OscilloscopePanel = OscilloscopePanel()
        self.SimulationPanel = SimulationPanel()
        self.PlaceHolder = QWidget()
        
        ControlPanel = QWidget()
        ControlPanelLayout = QVBoxLayout(ControlPanel)
        ControlPanelLayout.setContentsMargins(0, 0, 0, 0)
        ControlPanelLayout.setSpacing(0)
        
        ControlPanelLayout.addWidget(self.OscilloscopePanel, 3) #Reminder to tweak these later (TODO)
        ControlPanelLayout.addWidget(self.SimulationPanel, 1)
        ControlPanelLayout.addWidget(self.PlaceHolder, 6)
        
        #--------------------------------------------------#
        
        MainPanel = QWidget()
        MainPanelLayout = QHBoxLayout(MainPanel)
        MainPanelLayout.setContentsMargins(0, 0, 0, 0)
        MainPanelLayout.setSpacing(0)
 
        MainPanelLayout.addWidget(PalettePanel, 3) #These ones too ^^^^^
        MainPanelLayout.addWidget(CanvasPanel, 14)
        MainPanelLayout.addWidget(ControlPanel, 4)
    
        #---------------------------------------------------#
    
        ScreenPanel = QWidget()
        ScreenPanelLayout = QVBoxLayout(ScreenPanel)
        ScreenPanelLayout.setContentsMargins(0, 0, 0, 0)
        ScreenPanelLayout.setSpacing(0)
 
        ScreenPanelLayout.addWidget(self.ToolBar, 2)
        ScreenPanelLayout.addWidget(MainPanel, 24)
        ScreenPanelLayout.addWidget(self.StatusBar, 1)
 
        ScreenPanel.setStyleSheet("background-color: #001733;")
 
        self.setCentralWidget(ScreenPanel)
    
Application = QApplication(sys.argv)
Window = MainWindow()
Window.show()
sys.exit(Application.exec())
