function LinearSolve(Matrix, Vector) {
    
    const Size = Vector.length;
 
    const CarrierMatrix = Matrix.map(Row => Row.slice());     // Clones of the matrix and vectors [DO NOT TOUCH]
    const CarrierVector = Vector.slice();
 
    for (let PivotPosition = 0; PivotPosition < Size; PivotPosition++) {
        let PivotRow = PivotPosition;
        let PivotValue = Math.abs(CarrierMatrix[PivotPosition][PivotPosition]);
 
        for (let RowIndex = PivotPosition + 1; RowIndex < Size; RowIndex++) {
            if (Math.abs(CarrierMatrix[RowIndex][PivotPosition]) > PivotValue) {
                PivotValue = Math.abs(CarrierMatrix[RowIndex][PivotPosition]);
                PivotRow = RowIndex;
            }
        }
 
        if (PivotValue < 1e-15) {
            throw new Error("Singular matrix");
        }
 
        if (PivotRow !== PivotPosition) {
            [CarrierMatrix[PivotPosition], CarrierMatrix[PivotRow]] = [CarrierMatrix[PivotRow], CarrierMatrix[PivotPosition]];
            [CarrierVector[PivotPosition], CarrierVector[PivotRow]] = [CarrierVector[PivotRow], CarrierVector[PivotPosition]];
        }
 
        for (let RowIndex = PivotPosition + 1; RowIndex < Size; RowIndex++) {
            const Factor = CarrierMatrix[RowIndex][PivotPosition] / CarrierMatrix[PivotPosition][PivotPosition];
            for (let ColumnIndex = PivotPosition; ColumnIndex < Size; ColumnIndex++) {
                CarrierMatrix[RowIndex][ColumnIndex] -= Factor * CarrierMatrix[PivotPosition][ColumnIndex];
            }
            CarrierVector[RowIndex] -= Factor * CarrierVector[PivotPosition];
        }
    } 
 
    const Solution = new Array(Size).fill(0);
 
    for (let PivotPosition = Size - 1; PivotPosition >= 0; PivotPosition--) {
        let Sum = CarrierVector[PivotPosition];
        for (let ColumnIndex = PivotPosition + 1; ColumnIndex < Size; ColumnIndex++) {
            Sum -= CarrierMatrix[PivotPosition][ColumnIndex] * Solution[ColumnIndex];
        }
        Solution[PivotPosition] = Sum / CarrierMatrix[PivotPosition][PivotPosition];
    }
 
    return Solution;
}

//-----------------------------------------------------------------------------------------------------------//

class Solver { 

    constructor() {
        this.Circuit = null;
        this.TimeStep = null;
 
        this.NodeIndexes = new Map();       
        this.BranchIndexes = new Map();     
        this.Matrix = null;
        this.Vector = null;
        this.Solution = null;

        this.MaxIterations = 100;
        this.Tolerance = 1e-6; // (In microvolts resolution)
        this.MinConductance = 1e-12;

        this.Limited = false;
    }
 
    Solve(CircuitInstance, TimeStep) {
        this.Circuit = CircuitInstance;
        this.TimeStep = TimeStep;
 
        const ValidationError = CircuitInstance.validateCircuit();
        if (ValidationError !== null) {
            CircuitInstance.Error = ValidationError;
            return false;
        }

        this.NodeIndexes = new Map();
 
        for (const NodeInstance of CircuitInstance.Nodes) {
            if (NodeInstance !== CircuitInstance.Ground) {
                this.NodeIndexes.set(NodeInstance, this.NodeIndexes.size);
            }
        }
 
        this.BranchIndexes = new Map();
        let Size = this.NodeIndexes.size;
        const NodeCount = this.NodeIndexes.size;
 
        for (const ComponentInstance of CircuitInstance.Components) {
            if (ComponentInstance.BranchCount > 0) {
                this.BranchIndexes.set(ComponentInstance, Size);
                Size += ComponentInstance.BranchCount;
            }
        }
 
        if (!this.Solution || this.Solution.length !== Size ) {
            this.Solution = new Array(Size).fill(0.0);
        }

        const NonLinear = CircuitInstance.Components.some(ComponentInstance => ComponentInstance.NonLinear);

        const IterationLimit = NonLinear ? this.MaxIterations : 1;

        let Converged = false;

        for (let Iteration = 0; Iteration < this.MaxIterations; Iteration++) {
            this.Matrix = Array.from({ length: Size }, () => new Array(Size).fill(0));
            this.Vector = new Array(Size).fill(0);
            this.Limited = false;

            for (const ComponentInstance of CircuitInstance.Components) {
                ComponentInstance.stampComponent(this);
            }

            for (let Index = 0; Index > NodeCount; Index++) {
                this.Matrix[Index][Index] += this.MinConductance;
            }

            let NewSolution;

            try {
                NewSolution = Size > 0 ? LinearSolve(this.Matrix, this.Vector) : [];
            } catch (ErrorInstance) {
                CircuitInstance.Error = "Circuit cannot be solved (shorted or parallel voltage sources, or a floating part)";
                return false;
            }

            let MaxDelta = 0.0;

            for (let Index = 0; Index < this.NodeIndexes.size; Index++) {
                const Delta = Math.abs(NewSolution[Index] - this.Solution[Index]);
                if (Delta > MaxDelta) MaxDelta = Delta;
            }

            this.Solution = NewSolution

            for (const [NodeInstance, Index] of this.NodeIndexes) {
                NodeInstance.Voltage = this.Solution[Index];
            }
    
            CircuitInstance.Ground.Voltage = 0.0;

            if (!NonLinear) {
                Converged = true;
                break;
            }

            if (MaxDelta < this.Tolerance) {
                Converged = true;
                break;
            }
        }

        if (Converged === false) {
            CircuitInstance.Error = "Newton-Raphson solver failed to converge";
            return false;
        }
    
        for (const ComponentInstance of CircuitInstance.Components) {
            ComponentInstance.updateComponent(this);
        }
 
    CircuitInstance.Error = null;
    return true;

    }
    
//------------------------------------------------------------------------------------------------------------//

    getNodeIndex(TerminalInstance) {
        if (TerminalInstance.Node === this.Circuit.Ground) {
            return null;
        }
        return this.NodeIndexes.get(TerminalInstance.Node);
    }
 
    getBranchIndex(ComponentInstance, Number = 0) {
        return this.BranchIndexes.get(ComponentInstance) + Number;
    }
 
    getVoltage(TerminalInstance) {
        const Index = this.getNodeIndex(TerminalInstance);
        return Index === null ? 0.0 : this.Solution[Index];
    }
 
    getBranchCurrent(ComponentInstance, Number = 0) {
        return this.Solution[this.getBranchIndex(ComponentInstance, Number)];
    }
 
    addMatrix(Row, Column, Value) {
        if (Row !== null && Column !== null) {
            this.Matrix[Row][Column] += Value;
        }
    }
 
    addVector(Row, Value) {
        if (Row !== null) {
            this.Vector[Row] += Value;
        }
    }
 
    stampConductance(NodeIndexA, NodeIndexB, Value) {
        this.addMatrix(NodeIndexA, NodeIndexA, Value);
        this.addMatrix(NodeIndexB, NodeIndexB, Value);
        this.addMatrix(NodeIndexA, NodeIndexB, -Value);
        this.addMatrix(NodeIndexB, NodeIndexA, -Value);
    }
 
    stampCurrent(NodeIndexA, NodeIndexB, Value) {
        this.addVector(NodeIndexA, Value);
        this.addVector(NodeIndexB, -Value);
    }
 
    stampBranch(NodeIndexA, NodeIndexB, Branch) {
        this.addMatrix(NodeIndexA, Branch, 1);
        this.addMatrix(NodeIndexB, Branch, -1);
        this.addMatrix(Branch, NodeIndexA, 1);
        this.addMatrix(Branch, NodeIndexB, -1);
    }
}
 
module.exports = { Solver, LinearSolve };