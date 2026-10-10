const { Node, Circuit, Connection} = require("./Topology.js")
const { Solver } = require("./Solver.js") 

class Scene {
    constructor() {
        this.Components = new Set();
        this.Connections = new Set();

        this.Nodes = [];
        this.Circuits = [];

        this.Solver = new Solver();
        this.TimeStep = 0.01;
        this.SimulationTime = 0.0;

        this.UnresolvedTopology = true;
        this.Errors = [];
    }

    addComponent(ComponentInstance) {
        this.Components.add(ComponentInstance);
        ComponentInstance.Scene = this;
        this.UnresolvedTopology = true;
    }

    removeComponent(ComponentInstance) {
        for (const TerminalInstance of ComponentInstance.Terminals) {
            for (const ConnectionInstance of [...TerminalInstance.Connections]) {
                this.removeConnection(TerminalInstance, ConnectionInstance.getRemoteTerminal(TerminalInstance));
            }
        }
        this.Components.delete(ComponentInstance);
        ComponentInstance.Scene = null;
        this.UnresolvedTopology = true;
    }

    addConnection(TerminalA, TerminalB) {
        const NewConnection = new Connection(TerminalA, TerminalB);
        TerminalA.Connections.add(NewConnection);
        TerminalB.Connections.add(NewConnection);
        this.Connections.add(NewConnection);
        this.UnresolvedTopology = true;
    }

    removeConnection(TerminalA, TerminalB) {
        for (const ConnectionInstance of [...TerminalA.Connections]) {
            if (ConnectionInstance.getRemoteTerminal(TerminalA) === TerminalB) {
                this.Connections.delete(ConnectionInstance);
                TerminalA.Connections.delete(ConnectionInstance);
                TerminalB.Connections.delete(ConnectionInstance);
            }
        }
        this.UnresolvedTopology = true;
    }
    
    updateScene() {
        if (this.UnresolvedTopology === true) {

            for (const ComponentInstance of this.Components) {
                ComponentInstance.Circuit = null;
                for (const TerminalInstance of ComponentInstance.Terminals) {
                    TerminalInstance.Node = null;
                }
            }

            this.Nodes = [];
 
            for (const ComponentInstance of this.Components) {
                for (const TerminalInstance of ComponentInstance.Terminals) {
                    if (TerminalInstance.Node === null) {
                        const NewNode = new Node();
                        NewNode.updateNode(TerminalInstance);
                        this.Nodes.push(NewNode);
                    }
                }
            }
 
            this.Circuits = [];
 
            for (const NodeInstance of this.Nodes) {
                if (NodeInstance.Circuit === null) {
                    const NewCircuit = new Circuit();
                    NewCircuit.updateCircuit(NodeInstance);
                    this.Circuits.push(NewCircuit);
                }
            }
 
            this.UnresolvedTopology = false;
        }
    }

    BackendSimulationStep(Steps = 1) {
        this.updateScene();

        for (let Step = 0; Step < Steps; Step++) {
            this.Errors = [];

            for (const CircuitInstance of this.Circuits) {
                if (!this.Solver.Solve(CircuitInstance, this.TimeStep)) {
                    this.Errors.push(CircuitInstance.Error);
                }
            }
            if (this.Errors.length > 0) {
                return false;
            }
            this.SimulationTime += this.TimeStep
        }
        return true;
    }
}

module.exports = { Scene };