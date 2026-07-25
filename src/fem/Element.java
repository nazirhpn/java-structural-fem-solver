package fem;

import iceb.jnumerics.IMatrix;
import iceb.jnumerics.IMatrixRO;
import iceb.jnumerics.IVector;
import iceb.jnumerics.Vector3D;
import iceb.jnumerics.Array2DMatrix;
import iceb.jnumerics.ArrayVector;

public class Element {
    private double Area;
    private double eModulus;
    private double length;
    private double thermalConductivity;  // Thermal conductivity for thermal analysis
    private double thermalFlux;          // Thermal flux applied to the element
    private double alpha = 0;  // Coefficient of thermal expansion for Steel (1/°C)
    private int[] dofNumbers = new int[6]; // Degrees of freedom for the element (3 per node)
    private Node node1;
    private Node node2;

    // Constructor
    public Element(double e, double a, Node n1, Node n2) {
        this.Area = a;
        this.eModulus = e;
        this.node1 = n1;
        this.node2 = n2;
    }

    // Getter methods for core properties
    public double getLength() {
        Vector3D posi = node1.getPosition().subtract(node2.getPosition());
        this.length = posi.normTwo();
        return this.length;
    }

    public Node getNode1() {
        return this.node1;
    }

    public Node getNode2() {
        return this.node2;
    }

    public double getArea() {
        return this.Area;
    }

    public double getEModulus() {
        return this.eModulus;
    }

    public int[] getDOFNumb() {
        return dofNumbers;
    }

    // Enumeration of Degrees of Freedom (DOFs) for both nodes
    public void enumerateDOFs() {
        int[] dofnumbers1 = node1.getDOFNumb();
        int[] dofnumbers2 = node2.getDOFNumb();
        for (int i = 0; i < 3; i++) {
            dofNumbers[i] = dofnumbers1[i];
            dofNumbers[i + 3] = dofnumbers2[i];
        }
    }

    // Structural Stiffness Matrix (Elastic)
    public IMatrix computeStiffnessMatrix() {
        double constant = (this.eModulus * this.Area) / Math.pow(this.getLength(), 3);
        return createStiffnessMatrixWithConstant(constant);
    }

    protected IMatrix createStiffnessMatrixWithConstant(double constant) {
        IMatrix stiffnessMatrix = new Array2DMatrix(6, 6);
        Vector3D n1 = this.node1.getPosition();
        Vector3D n2 = this.node2.getPosition();
        IMatrixRO dyadic1 = ((n1.subtract(n2)).dyadicProduct(n1.subtract(n2))).multiply(constant);
        IMatrixRO dyadic2 = ((n1.subtract(n2)).dyadicProduct(n2.subtract(n1))).multiply(constant);
        IMatrixRO dyadic3 = ((n2.subtract(n1)).dyadicProduct(n1.subtract(n2))).multiply(constant);
        IMatrixRO dyadic4 = ((n2.subtract(n1)).dyadicProduct(n2.subtract(n1))).multiply(constant);
        stiffnessMatrix.setMatrix(0, 0, dyadic1);
        stiffnessMatrix.setMatrix(0, 3, dyadic2);
        stiffnessMatrix.setMatrix(3, 0, dyadic3);
        stiffnessMatrix.setMatrix(3, 3, dyadic4);
        return stiffnessMatrix;
    }

    // Geometric Stiffness Matrix (Non-linear part)
    public IMatrix computeGeometricStiffnessMatrix() {
        double axialForce = this.computeAxialForce();
        IMatrix kGeom = new Array2DMatrix(6, 6);
        double L = this.getLength();
        double factor = axialForce / (L * L);

        // Assuming 1D truss elements for simplicity
        kGeom.set(0, 0, factor);
        kGeom.set(0, 3, -factor);
        kGeom.set(3, 0, -factor);
        kGeom.set(3, 3, factor);

        return kGeom;
    }

    // Thermal Stiffness Matrix Calculation
    public IMatrix computeThermalStiffnessMatrix() {
        IMatrix kLocal = new Array2DMatrix(2, 2);
        double L = this.getLength();
        double k = this.thermalConductivity * this.Area / L;

        kLocal.set(0, 0, k);
        kLocal.set(0, 1, -k);
        kLocal.set(1, 0, -k);
        kLocal.set(1, 1, k);

        return transformToGlobal(kLocal);  // Transform to global coordinate system
    }

    public IMatrix computeThermoMechanicalCouplingMatrix() {
        IMatrix K_u_theta = new Array2DMatrix(6, 2);  // 6 DOFs (displacement) and 2 DOFs (temperature)

        Vector3D diff = node2.getPosition().subtract(node1.getPosition());
        double L = diff.normTwo();

        // Compute the coupling matrix (assuming a simple linear dependence)
        double alpha = this.alpha;  // Get the thermal expansion coefficient
        double factor = alpha * this.eModulus * this.Area / L;

        for (int i = 0; i < 3; i++) {
            K_u_theta.set(i, 0, -factor * diff.get(i));  // Node 1 contributions
            K_u_theta.set(i + 3, 1, factor * diff.get(i));  // Node 2 contributions
        }
//        // Debugging: print the final K_u_theta matrix
//        System.out.println("Thermo-Mechanical Coupling Matrix (K_u_theta):");
//        for (int i = 0; i < K_u_theta.getRowCount(); i++) {
//            for (int j = 0; j < K_u_theta.getColumnCount(); j++) {
//                System.out.print(K_u_theta.get(i, j) + "\t");
//            }
//            System.out.println();
//        }
        return K_u_theta;
    }
    
    // Calculate the thermal strain based on the temperature difference
    public double computeThermalStrain() {
    	double alpha = this.alpha;
        double T1 = this.node1.getTemperature();
        double T2 = this.node2.getTemperature();
        double deltaT = ((T1 + T2) / 2) - Structure.getRoomTemperature();  // Average temperature difference
        return alpha * deltaT;
    }

    // Modify the axial force computation to include thermal strain
    public double computeAxialForce() {
        Vector3D displacement1 = this.node1.getDisplacement();  // Current displacement of Node 1
        Vector3D displacement2 = this.node2.getDisplacement();  // Current displacement of Node 2
        Vector3D relativeDisplacement = displacement1.subtract(displacement2);  // Relative displacement

        Vector3D originalDirection = this.node2.getPosition().subtract(this.node1.getPosition()).normalize();  // Original direction
        double L = this.getLength();  // Length of the element

        // Compute mechanical strain
        double mechanicalStrain = relativeDisplacement.dot(originalDirection) / L;
        double totalStrain = mechanicalStrain;
        
        // Total strain = mechanical strain + thermal strain
        if (this.alpha != 0) {
        totalStrain = mechanicalStrain + computeThermalStrain();
        }
        // Axial force = E * A * totalStrain
        return this.eModulus * totalStrain * this.Area;
    }

    // Load Vector
    public IVector computeLoadVector() {
        IVector rLocal = new ArrayVector(6);
        if (this.node1.getForce() != null) {
            rLocal.set(0, this.node1.getForce().getComponents(0));
            rLocal.set(1, this.node1.getForce().getComponents(1));
            rLocal.set(2, this.node1.getForce().getComponents(2));
        }
        if (this.node2.getForce() != null) {
            rLocal.set(3, this.node2.getForce().getComponents(0));
            rLocal.set(4, this.node2.getForce().getComponents(1));
            rLocal.set(5, this.node2.getForce().getComponents(2));
        }
        return rLocal;
    }
    
    // Non-linear internal force vector (geometric + material stiffness)
    public double[] computeInternalForceVector() {
        double[] internalForce = new double[6];  // 3 DOFs per node, 2 nodes

        Vector3D u1 = getNode1().getDisplacement();
        Vector3D u2 = getNode2().getDisplacement();

        // Initial and current length in 3D
        double initialLength = getLength();
        Vector3D relativeDisplacement = u2.subtract(u1);
        double currentLength = relativeDisplacement.normTwo();

        // Green-Lagrange strain for 3D truss
        double strain = 0.5 * ((currentLength * currentLength) - (initialLength * initialLength)) / (initialLength * initialLength);
        double stress = getEModulus() * strain;

        // Compute internal forces in 3D (based on stress)
        for (int i = 0; i < 3; i++) {
            internalForce[i] = stress * (relativeDisplacement.get(i) / currentLength);
            internalForce[i + 3] = -internalForce[i];  // Opposite force for node 2
        }

        return internalForce;
    }
	
    // Transform Local Stiffness Matrix to Global Coordinates
    private IMatrix transformToGlobal(IMatrix kLocal) {
        Vector3D pos1 = this.node1.getPosition();
        Vector3D pos2 = this.node2.getPosition();
        Vector3D diff = pos2.subtract(pos1);
        double L = diff.normTwo();

        if (L == 0) {
            System.out.println("Error: Length of element is zero in transformToGlobal.");
            return kLocal;
        }

        // Compute direction cosines
        double l = diff.getX1() / L;
        double m = diff.getX2() / L;
        double n = diff.getX3() / L;

        // Create transformation matrix for 3D truss
        IMatrix T_matrix = new Array2DMatrix(2, 6);
        T_matrix.set(0, 0, l);
        T_matrix.set(0, 1, m);
        T_matrix.set(0, 2, n);
        T_matrix.set(1, 3, l);
        T_matrix.set(1, 4, m);
        T_matrix.set(1, 5, n);

        IMatrix T_transpose = transpose(T_matrix);
        IMatrix tempMatrix = multiplyMatrices(T_transpose, kLocal);
        return multiplyMatrices(tempMatrix, T_matrix);
    }

    // Matrix Transposition
    private IMatrix transpose(IMatrix original) {
        int rows = original.getRowCount();
        int cols = original.getColumnCount();
        IMatrix transpose = new Array2DMatrix(cols, rows);
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                transpose.set(j, i, original.get(i, j));
            }
        }
        return transpose;
    }

    // Matrix Multiplication
    private IMatrix multiplyMatrices(IMatrix A, IMatrix B) {
        int rowsA = A.getRowCount();
        int colsA = A.getColumnCount();
        int rowsB = B.getRowCount();
        int colsB = B.getColumnCount();

        if (colsA != rowsB) {
            throw new IllegalArgumentException("Matrix dimensions do not allow multiplication.");
        }

        IMatrix result = new Array2DMatrix(rowsA, colsB);
        for (int i = 0; i < rowsA; i++) {
            for (int j = 0; j < colsB; j++) {
                double sum = 0;
                for (int k = 0; k < colsA; k++) {
                    sum += A.get(i, k) * B.get(k, j);
                }
                result.set(i, j, sum);
            }
        }
        return result;
    }
    
	public Vector3D getDisplacementOfNode1() {
	    Vector3D originalPosition1 = this.getNode1().getPosition();
	    Vector3D displacements1 = this.getNode1().getDisplacement().multiply(Visualizer.displacementScale);
	    Vector3D point1 = originalPosition1.add(displacements1);
	    return point1;
	}
	
	public Vector3D getDisplacementOfNode2() {
	    Vector3D originalPosition2 = this.getNode2().getPosition();
	    Vector3D displacements2 = this.getNode2().getDisplacement().multiply(Visualizer.displacementScale);
	    Vector3D point2 = originalPosition2.add(displacements2);
	    return point2;
	}
	
    // Set thermal conductivity separately
    public void setThermalConductivity(double k) {
        this.thermalConductivity = k;
    }

    public void setThermalFlux(double flux) {
        this.thermalFlux = flux;
    }

    public double getThermalFlux() {
        return this.thermalFlux;
    }
    
    public void setAlpha(double alpha) {
        this.alpha = alpha;
    }

    public double getAlpha() {
        return this.alpha;
    }
    
	public void print() {
		System.out.println(new Vector3D(this.eModulus, this.Area, this.length));
	}
}
