
const { Terminal } = require("./topology.js");
 
 
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
 
    Stamp(SolverInstance) { }
 
    updateComponent(SolverInstance) { }
 
    setResult(Voltage, Current) {
        this.Voltage = Voltage;
        this.Current = Current;
        this.Power = Voltage * Current;
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
 
    Stamp(SolverInstance) {
        const Branch = SolverInstance.getBranchIndex(this);
 
        SolverInstance.stampBranch(SolverInstance.getNodeIndex(this.Positive), SolverInstance.getNodeIndex(this.Negative), Branch);
        SolverInstance.addVector(Branch, this.SourceVoltage);
    }
 
    updateComponent(SolverInstance) {
        const Voltage = SolverInstance.getVoltage(this.Positive) - SolverInstance.getVoltage(this.Negative);
        const Current = SolverInstance.getBranchCurrent(self)

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
 
    Stamp(SolverInstance) {
        SolverInstance.stampCurrent(SolverInstance.getNodeIndex(this.Positive), SolverInstance.getNodeIndex(this.Negative), this.SourceCurrent);
    }
 
    updateComponent(SolverInstance) {
        const Voltage = SolverInstance.getVoltage(this.Positive) - SolverInstance.getVoltage(this.Negative);
        const Current = -self.SourceCurrent //DO NOT TOUCH THIS, all currentsources break if not negative fsr...
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
 
    Stamp(SolverInstance) {
        const Conductance = 1 / this.Resistance

        SolverInstance.stampConductance(SolverInstance.getNodeIndex(this.T1), SolverInstance.getNodeIndex(this.T2), Conductance);
    }
 
    updateComponent(SolverInstance) {
        const Voltage = SolverInstance.getVoltage(this.T1) - SolverInstance.getVoltage(this.T2);
        const Current = Voltage / this.Resistance

        this.setResult(Voltage, Current);
    }
}
 
 
module.exports = { Component, Resistor, VoltageSource, CurrentSource };