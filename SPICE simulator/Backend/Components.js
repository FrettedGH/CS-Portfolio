const { Terminal } = require("./Topology.js");
 
class Component {

    BranchCount = 0; //How many extra unknowns 
    NonLinear = false;
 
    constructor() {
        this.Terminals = [];
        this.Scene = null;
        this.Circuit = null;     
 
        this.Voltage = null;
        this.Current = null;
        this.Power = null;
    }

    stampComponent(SolverInstance) { }
 
    updateComponent(SolverInstance) { }

    validateComponent() { }

    addTerminal() {
        const NewTerminal = new Terminal(this);
        this.Terminals.push(NewTerminal);
        return NewTerminal;
    }
 
    setResult(Voltage, Current) {
        this.Voltage = Voltage;
        this.Current = Current;
        this.Power = Voltage * Current;
    }

    resetState() {
        this.Voltage = 0.0
        this.Current = 0.0
        this.Power = 0.0
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

    validateComponent() {
        if (this.SourceVoltage === null) {
            return "Voltage source has no voltage set";
        }
        return null;
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

    validateComponent() {
        if (this.SourceCurrent === null) {
            return "Current source has no current set";
        }
        return null;
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

    validateComponent() {
        if (this.Resistance === null || this.Resistance <= 0) {
            return "Resistance must be greater than 0";
        }
        return null;
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

    validateComponent() {
        if (this.Capacitance === null || this.Capacitance <= 0) {
            return "Capacitance must be greater than 0";
        }
        return null;
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
        super().resetState();
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

    validateComponent() {
        if (this.Inductance === null || this.Inductance <= 0) {
            return "Inductance must be greater than 0";
        }
        return null;
    }


    stampComponent(SolverInstance) {
        const Branch = SolverInstance.getBranchIndex(this);
        const Resistance = this.Inductance / SolverInstance.TimeStep;

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
        super().resetState();
        this.PreviousCurrent = 0.0;
    }
}

class Diode extends Component {
    NonLinear = true;
    constructor() {
        super();

        this.Anode = this.addTerminal();
        this.Cathode = this.addTerminal();

        this.SaturationCurrent = 1e-14;
        this.Ideality = 1.0;
        this.ThermalVoltage = 0.02585;

        this.IterationVoltage = 0.0;
    }

    validateComponent() {
        if (!(this.SaturationCurrent > 0 && this.Ideality > 0 && this.ThermalVoltage > 0)) {
            return "Diode parameters must be greater than 0";
        }
        return null;
    }

    stampComponent(SolverInstance) { //Schockley's equation
        const Factor = this.ThermalVoltage * this.Ideality;
        const CritcialVoltage = Factor * Math.log(Factor / (Math.SQRT2 * this.SaturationCurrent));

        let Voltage = SolverInstance.getVoltage(this.Anode) - SolverInstance.getVoltage(this.Cathode);

        if (Voltage > CriticalVoltage && Math.abs(Voltage - this.IterationVoltage) > 2 * Factor) {
            if (this.IterationVoltage > 0) {
                const Argument = 1 + (Voltage - this.IterationVoltage) / Factor;
                Voltage = Argument > 0 ? this.IterationVoltage + Factor * Math.log(Argument) : CriticalVoltage;
            } else {
                Voltage = Factor * Math.log(Voltage / Factor);
            }
            SolverInstance.Limited = true;
        }

        this.IterationVoltage = Voltage;

        const Exponential = Math.exp(Voltage / Factor);
        const Current = this.SaturationCurrent * (Exponential - 1);
        const Conductance = (this.SaturationCurrent / Factor) * Exponential;
        const Equivalent = Current - Conductance * Voltage;

        const NodeA = SolverInstance.getNodeIndex(this.Anode);
        const NodeB = SolverInstance.getNodeIndex(this.Cathode);

        SolverInstance.stampConductance(NodeA, NodeB, Conductance);
        SolverInstance.stampCurrent(NodeA, NodeB, -Equivalent);
    }

    updateComponent(SolverInstace) {
        const Factor = this.Ideality * this.ThermalVoltage;
        const Voltage = SolverInstance.getVoltage(this.Anode) - SolverInstance.getVoltage(this.Cathode);
        const Current = this.SaturationCurrent * (Math.exp(Math.min(Voltage / Factor, 100)) - 1);

        this.setResult(Voltage, Current);
    }

    resetState() {
        super.resetState();
        this.IterationVoltage = 0.0;
    }

}

module.exports = { Component, VoltageSource, CurrentSource, Resistor, Capacitor, Inductor, Diode };