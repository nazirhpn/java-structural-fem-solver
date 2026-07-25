package fem;

import iceb.jnumerics.ArrayVector;
import iceb.jnumerics.IVector;
import iceb.jnumerics.Vector3D;

public class Node {
    private int[] dofNumbers = new int[3];
    private Vector3D position;
    private Vector3D displacement;
    private Constraint constraints;
    private Force forces;
    private double temperature;  // Field to store temperature
    private Double prescribedTemperature = null;  // Prescribed temperature (if any)

    // Constructor to initialize the position and set initial displacement to zero
    public Node(double x1, double x2, double x3) {
        this.position = new Vector3D(x1, x2, x3);
        this.displacement = new Vector3D(0.0, 0.0, 0.0);
    }

    // Setters and Getters for Constraints, Forces, and DOF Enumeration
    public void setConstraint(Constraint c) {
        this.constraints = c;
    }

    public Constraint getConstraint() {
        return this.constraints;
    }

    public void setForce(Force f) {
        this.forces = f;
    }

    public Force getForce() {
        return this.forces;
    }

    public int enumerateDOFs(int currentDOF) {
        if (constraints != null) {
            for (int j = 0; j < dofNumbers.length; j++) {
                if (constraints.isFree(j)) {
                    dofNumbers[j] = currentDOF++;
                } else {
                    dofNumbers[j] = -1;
                }
            }
        } else {
            for (int j = 0; j < dofNumbers.length; j++) {
                dofNumbers[j] = currentDOF++;
            }
        }
        return currentDOF;
    }

    public int[] getDOFNumb() {
        return dofNumbers;
    }

    // Compute load vector based on applied forces
    public IVector computeLoadVector() {
        IVector rLocal = new ArrayVector(6);
        if (this.forces != null) {
            rLocal.set(0, this.forces.getComponents(0));
            rLocal.set(1, this.forces.getComponents(1));
            rLocal.set(2, this.forces.getComponents(2));
        }
        return rLocal;
    }

    // Position and Displacement Management
    public Vector3D getPosition() {
        return this.position;
    }

    public void setDisplacement(double[] u) {
        this.displacement = new Vector3D(u[0], u[1], u[2]);
    }

    public Vector3D getDisplacement() {
        return this.displacement;
    }

    // Temperature management for thermal analysis
    public void setTemperature(double temp) {
        this.temperature = temp;
    }

    public double getTemperature() {
        return this.temperature;
    }

    public void setPrescribedTemperature(double temp) {
        this.prescribedTemperature = temp;
    }

    public boolean isTemperaturePrescribed() {
        return this.prescribedTemperature != null;
    }

    public double getPrescribedTemperature() {
        if (this.prescribedTemperature != null) {
            return this.prescribedTemperature;
        }
        throw new IllegalStateException("No prescribed temperature set for this node.");
    }

    public void print() {
        System.out.println(this.position);
    }
}
