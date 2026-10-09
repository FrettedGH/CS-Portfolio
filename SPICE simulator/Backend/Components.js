const { Terminal } = require("./Topology.js");
 
class Component {

    BranchCount = 0; //How many extra unknowns
 
    constructor() {
        this.Terminals = [];
        this.Scene = null;
        this.Circuit = null;     
 
        this.Voltage = null;
        this.Current = null;
        this.Power = null;
    }
 
    addTerminal() {
        const NewTerminal = new Terminal(this);
        this.Terminals.push(NewTerminal);
        return NewTerminal;
    }
 
    stampComponent(SolverInstance) { }
 
    updateComponent(SolverInstance) { }
 
    setResult(Voltage, Current) {
        this.Voltage = Voltage;
        this.Current = Current;
        this.Power = Voltage * Current;
    }

    resetState() {
        self.Voltage = 0.0
        self.Current = 0.0
        self.Power = 0.0
    }
}
 
class VoltageSource extends Component {
    BranchCount = 1;
 
    constructor() {
        super();
 
        this.Positive = this.addTerminal(); // Terminals[0]
        this.Negative = this.addTerminal(); // Terminals[1]
 
        this.SourceVoltage = null;                
    }
 
    setVoltage(Voltage) {
        this.SourceVoltage = Voltage;
    }
 
    stampComponent(SolverInstance) {
        const Branch = SolverInstance.getBranchIndex(this);
 
        SolverInstance.stampBranch(SolverInstance.getNodeIndex(this.Positive), SolverInstance.getNodeIndex(this.Negative), Branch);
        SolverInstance.addVector(Branch, this.SourceVoltage);
    }
 
    updateComponent(SolverInstance) {
        const Voltage = SolverInstance.getVoltage(this.Positive) - SolverInstance.getVoltage(this.Negative);
        const Current = SolverInstance.getBranchCurrent(this)

        this.setResult(Voltage, Current);
    }
}
 
class CurrentSource extends Component {
    constructor() {
        super();
 
        this.Positive = this.addTerminal();
        this.Negative = this.addTerminal();
 
        this.SourceCurrent = null;                
    } 
 
    setCurrent(Current) {
        this.SourceCurrent = Current;
    }
 
    stampComponent(SolverInstance) {
        SolverInstance.stampCurrent(SolverInstance.getNodeIndex(this.Positive), SolverInstance.getNodeIndex(this.Negative), this.SourceCurrent);
    }
 
    updateComponent(SolverInstance) {
        const Voltage = SolverInstance.getVoltage(this.Positive) - SolverInstance.getVoltage(this.Negative);
        const Current = -this.SourceCurrent //DO NOT TOUCH THIS, all currentsources break if not negative fsr...

        this.setResult(Voltage, Current);     
    }
}

class Resistor extends Component {
    constructor() {
        super();
 
        this.T1 = this.addTerminal();
        this.T2 = this.addTerminal();
 
        this.Resistance = null;
    }
 
    setResistance(Resistance) {
        this.Resistance = Resistance;
    }
 
    stampComponent(SolverInstance) {
        const Conductance = 1 / this.Resistance

        SolverInstance.stampConductance(SolverInstance.getNodeIndex(this.T1), SolverInstance.getNodeIndex(this.T2), Conductance);
    }
 
    updateComponent(SolverInstance) {
        const Voltage = SolverInstance.getVoltage(this.T1) - SolverInstance.getVoltage(this.T2);
        const Current = Voltage / this.Resistance

        this.setResult(Voltage, Current);
    }
}

class Capacitor extends Component {
    constructor() {
        super();

        this.T1 = this.addTerminal();
        this.T2 = this.addTerminal();
 
        this.Capacitance = null;

        this.PreviousVoltage = 0.0;
    }

    setCapacitance(Capacitance) {
        this.Capacitance = Capacitance;
    }

    stampComponent(SolverInstance) {
        const Conductance = this.Capacitance / SolverInstance.TimeStep;
        const Equivalent = Conductance * this.PreviousVoltage;

        SolverInstance.stampConductance(SolverInstance.getNodeIndex(this.T1), SolverInstance.getNodeIndex(this.T2), Conductance);
        SolverInstance.stampCurrent(SolverInstance.getNodeIndex(this.T1), SolverInstance.getNodeIndex(this.T2), Equivalent);
    }

    updateComponent(SolverInstance) {
        const Voltage = SolverInstance.getVoltage(this.T1) - SolverInstance.getVoltage(this.T2);
        const Current = (this.Capacitance / SolverInstance.TimeStep) * (Voltage - this.PreviousVoltage);

        this.setResult(Voltage, Current);
        this.PreviousVoltage = Voltage;
    }

    resetState() {
        super();
        this.PreviousVoltage = 0.0;
    }
}

class Inductor extends Component {

    BranchCount = 1

    constructor() {
        super();

        this.T1 = this.addTerminal();
        this.T2 = this.addTerminal();
 
        this.Inductance = null;

        this.PreviousCurrent = 0.0;
    }

    setInductance(Inductance) {
        this.Inductance = Inductance;
    }

    stampComponent(SolverInstance) {
        const Branch = SolverInstance.getBranchIndex(this);
        const Resistance = Inductance * SolverInstance.TimeStep;

        SolverInstance.stampBranch(SolverInstance.getNodeIndex(this.T1), SolverInstance.getNodeIndex(this.T2), Branch);
        SolverInstance.addMatrix(Branch, Branch, -Resistance)
        SolverInstance.addVector(Branch, -Resistance * this.PreviousCurrent)
    }

    updateComponent(SolverInstance) {
        const Voltage = SolverInstance.getVoltage(this.T1) - SolverInstance.getVoltage(this.T2);
        const Current = SolverInstance.getBranchCurrent(this);

        this.setResult(Voltage, Current);
        this.PreviousCurrent = Current
    }

    resetState() {
        super();
        this.PreviousCurrent = 0.0;
    }
}

module.exports = { Component, Resistor, VoltageSource, CurrentSource };