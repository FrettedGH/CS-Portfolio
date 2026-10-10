class Circuit {
    constructor() { 
        this.Nodes = [];
        this.Components = [];
        this.Ground = null;      
        this.Error = null;       
    }

    validateCircuit() {
        for (const ComponentInstance of this.Components) {
            const Validation = ComponentInstance.validateComponent();
            if (Validation !== null) {
                return Validation;
            }
        }
        return null;
    }
 
    updateCircuit(StartingNode) {
        this.Nodes = [];
        this.Components = [];
        this.Ground = StartingNode;
 
        const VisitedNodes = new Set();
        const VisitedComponents = new Set();
 
        const NodeStack = [StartingNode];
 
        while (NodeStack.length > 0) {
            const CurrentNode = NodeStack.pop();
 
            if (!VisitedNodes.has(CurrentNode)) {
 
                VisitedNodes.add(CurrentNode);
                CurrentNode.Circuit = this;
                this.Nodes.push(CurrentNode);
 
                for (const TerminalInstance of CurrentNode.Terminals) {
                    const ComponentInstance = TerminalInstance.Component;
 
                    if (!VisitedComponents.has(ComponentInstance)) {
 
                        VisitedComponents.add(ComponentInstance);
                        ComponentInstance.Circuit = this;
                        this.Components.push(ComponentInstance);
 
                        for (const CounterpartTerminal of ComponentInstance.Terminals) {
                            NodeStack.push(CounterpartTerminal.Node);
                        }
                    }
                }
            }
        }
    }
} 

class Node {
    constructor() {
        this.Circuit = null;
        this.Terminals = [];
        this.Connections = [];
 
        this.Voltage = null;     // written by the Solver
    }
 
    updateNode(StartingTerminal) {
        this.Terminals = [];
        this.Connections = [];
 
        const VisitedTerminals = new Set();
        const VisitedConnections = new Set();
 
        const TerminalStack = [StartingTerminal];
 
        while (TerminalStack.length > 0) {
            const CurrentTerminal = TerminalStack.pop();
 
            if (!VisitedTerminals.has(CurrentTerminal)) {
 
                VisitedTerminals.add(CurrentTerminal);
                CurrentTerminal.Node = this;
                this.Terminals.push(CurrentTerminal);
 
                for (const ConnectionInstance of CurrentTerminal.Connections) {
                    if (!VisitedConnections.has(ConnectionInstance)) {
 
                        VisitedConnections.add(ConnectionInstance);
                        this.Connections.push(ConnectionInstance);
                    }
 
                    TerminalStack.push(ConnectionInstance.getRemoteTerminal(CurrentTerminal));
                }
            }
        }
    }
}
 
class Connection {
    constructor(TerminalA, TerminalB) {
        this.TerminalA = TerminalA;
        this.TerminalB = TerminalB;
    }
 
    getRemoteTerminal(TerminalInstance) {
        if (TerminalInstance === this.TerminalA) {
            return this.TerminalB;
        }
        if (TerminalInstance === this.TerminalB) {
            return this.TerminalA;
        }
        return null;
    }
}

class Terminal {
    constructor(ComponentInstance) {
        this.Component = ComponentInstance;
        this.Connections = new Set();   
        this.Node = null;               
    }
}

module.exports = { Terminal, Connection, Node, Circuit };