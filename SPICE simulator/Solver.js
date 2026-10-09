function LinearSolve(Matrix, Vector) {
    
    const Size = Vector.length;
 
    const Matrix = Matrix.map(Row => Row.slice());     // Clones of the matrix and vectors [DO NOT TOUCH]
    const Vector = Vector.slice();
 
    for (let PivotPosition = 0; PivotPosition < Size; PivotPosition++) {
        let PivotRow = PivotPosition;
        let PivotValue = Math.abs(Matrix[PivotPosition][PivotPosition]);
 
        for (let RowIndex = PivotPosition + 1; RowIndex < Size; RowIndex++) {
            if (Math.abs(Matrix[RowIndex][PivotPosition]) > PivotValue) {
                PivotValue = Math.abs(Matrix[RowIndex][PivotPosition]);
                PivotRow = RowIndex;
            }
        }
 
        if (PivotValue < 1e-15) {
            throw new Error("Singular matrix");
        }
 
        if (PivotRow !== PivotPosition) {
            [Matrix[PivotPosition], Matrix[PivotRow]] = [Matrix[PivotRow], Matrix[PivotPosition]];
            [Vector[PivotPosition], Vector[PivotRow]] = [Vector[PivotRow], Vector[PivotPosition]];
        }
 
        for (let RowIndex = PivotPosition + 1; RowIndex < Size; RowIndex++) {
            const Factor = Matrix[RowIndex][PivotPosition] / Matrix[PivotPosition][PivotPosition];
            for (let ColumnIndex = PivotPosition; ColumnIndex < Size; ColumnIndex++) {
                Matrix[RowIndex][ColumnIndex] -= Factor * Matrix[PivotPosition][ColumnIndex];
            }
            Vector[RowIndex] -= Factor * Vector[PivotPosition];
        }
    }
 
    const Solution = new Array(Size).fill(0);
 
    for (let PivotPosition = Size - 1; PivotPosition >= 0; PivotPosition--) {
        let Sum = Vector[PivotPosition];
        for (let ColumnIndex = PivotPosition + 1; ColumnIndex < Size; ColumnIndex++) {
            Sum -= Matrix[PivotPosition][ColumnIndex] * Solution[ColumnIndex];
        }
        Solution[PivotPosition] = Sum / Matrix[PivotPosition][PivotPosition];
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
    }
 
    Solve(CircuitInstance, TimeStep) {
        this.Circuit = CircuitInstance;
        this.TimeStep = TimeStep;
 
        this.NodeIndexes = new Map();
 
        for (const NodeInstance of CircuitInstance.Nodes) {
            if (NodeInstance !== CircuitInstance.Ground) {
                this.NodeIndexes.set(NodeInstance, this.NodeIndexes.size);
            }
        }
 
        this.BranchIndexes = new Map();
        let Size = this.NodeIndexes.size;
 
        for (const ComponentInstance of CircuitInstance.Components) {
            if (ComponentInstance.BranchCount > 0) {
                this.BranchIndexes.set(ComponentInstance, Size);
                Size += ComponentInstance.BranchCount;
            }
        }
 
        this.Matrix = Array.from({ length: Size }, () => new Array(Size).fill(0));
        this.Vector = new Array(Size).fill(0);
 
        for (const ComponentInstance of CircuitInstance.Components) {
            ComponentInstance.Stamp(this);
        }
 
        try {
            this.Solution = Size > 0 ? linSolve(this.Matrix, this.Vector) : [];
        } catch (Error_) {
            CircuitInstance.Error = "Circuit cannot be solved (shorted or parallel voltage sources, or a floating part)";
            return false;
        }
 
        for (const [NodeInstance, Index] of this.NodeIndexes) {
            NodeInstance.Voltage = this.Solution[Index];
        }
 
        CircuitInstance.Ground.Voltage = 0.0;
 
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
 
module.exports = { Solver, linSolve };