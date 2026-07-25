package fem;

import java.util.ArrayList;
import java.util.List;

import iceb.jnumerics.Array2DMatrix;
import iceb.jnumerics.IMatrix;
import iceb.jnumerics.IVector;
import iceb.jnumerics.MatrixFormat;
import iceb.jnumerics.QuadraticMatrixInfo;
import iceb.jnumerics.SolveFailedException;
import iceb.jnumerics.lse.GeneralMatrixLSESolver;
import iceb.jnumerics.lse.ILSESolver;
import inf.text.ArrayFormat;

public class Structure {

    private List<Node> nodes;       // List of nodes in the structure
    private List<Element> elements; // List of elements in the structure
    private static double roomTemperature = 20.0;  // Default room temperature in °C

    // Constructor
    public Structure() {
        this.nodes = new ArrayList<>();
        this.elements = new ArrayList<>();
    }

    // Node and Element Management
    public Node addNode(double x1, double x2, double x3) {
        Node n = new Node(x1, x2, x3);
        nodes.add(n);
        return n;
    }

    public Element addElement(double e, double a, Node n1, Node n2) {
        Element ele = new Element(e, a, n1, n2);
        elements.add(ele);
        return ele;
    }

    public int getNumberOfNodes() {
        return nodes.size();
    }

    public Node getNode(int id) {
        return nodes.get(id);
    }

    public int getNumberOfElements() {
        return elements.size();
    }

    public Element getElement(int id) {
        return elements.get(id);
    }

    public int enumerateDOFs() {
        int currentDOF = 0;
        for (Node node : nodes) {
            currentDOF = node.enumerateDOFs(currentDOF);
        }
        for (Element element : elements) {
            element.enumerateDOFs();
        }
        return currentDOF;
    }

    // Assembly Methods
    public void assembleStiffnessMatrix(IMatrix kGlobal, boolean withGeometric) {
        for (Element element : elements) {
            IMatrix kElastic = element.computeStiffnessMatrix();
            IMatrix kGeometric = null;

            if (withGeometric) {
                kGeometric = element.computeGeometricStiffnessMatrix();
            }

            int[] dofIndices = element.getDOFNumb();
            for (int i = 0; i < dofIndices.length; i++) {
                if (dofIndices[i] != -1) {  // Ensure DOF is free
                    for (int j = 0; j < dofIndices.length; j++) {
                        if (dofIndices[j] != -1) {  // Ensure DOF is free
                            kGlobal.add(dofIndices[i], dofIndices[j], kElastic.get(i, j));

                            if (withGeometric) {
                                kGlobal.add(dofIndices[i], dofIndices[j], kGeometric.get(i, j));
                            }
                        }
                    }
                }
            }
        }
    }

    public void assembleLoadVector(double[] rGlobal) {
        for (Node node : nodes) {
            IVector rLocal = node.computeLoadVector();
            int[] dofIndices = node.getDOFNumb();
            // Only saving the nodal forces that are free
            for (int i = 0; i < 3; i++) {
                if (dofIndices[i] != -1) {
                    rGlobal[dofIndices[i]] += rLocal.get(i);
                }
            }
        }
    }

    public void assembleInternalForceVector(double[] rGlobal) {
        for (Element element : elements) {
            double[] rLocal = element.computeInternalForceVector();
            int[] dofIndices = element.getDOFNumb();
            // Only saving the nodal forces that are free
            for (int i = 0; i < rLocal.length; i++) {
                if (dofIndices[i] != -1) {
                    rGlobal[dofIndices[i]] += rLocal[i];
                }
            }
        }
    }

    // Solver for mechanical part
    public void solve(boolean withGeometric) {
    	// Total number of DOFs
        int neq = this.enumerateDOFs();
        // Creates a linear system equation (LSE) solver for general matrices
        ILSESolver solver = new GeneralMatrixLSESolver();
        // Retrieves information about the matrix A (number of rows, columns, or symmetric)
        QuadraticMatrixInfo aInfo = solver.getAInfo();
        IMatrix a = solver.getA();
        double[] b = new double[neq];

        // Specifies that the matrix has neq x neq dimensions
        aInfo.setSize(neq);
        // Sets up internal data structures, allocates memory, and gets the solver ready
        solver.initialize();

        this.assembleStiffnessMatrix(a, withGeometric);
        this.assembleLoadVector(b);

        System.out.println("Solving A x = b");
        System.out.println("Matrix A");
        System.out.println(MatrixFormat.format(a));
        System.out.println("Vector b");
        System.out.println(ArrayFormat.format(b));

        try {
            solver.solve(b);  // Saves solution in b
        } catch (SolveFailedException e) {  // Matrix singularity, numerical issues, division by zero in factorization
            System.out.println("Solve failed: " + e.getMessage());
        }

        System.out.println("Solution x");
        System.out.println(ArrayFormat.format(b));
        // Saving the displacements
        this.selectDisplacements(b);
    }

//    public void solveNonLinear() {
//        double tolerance = 1e-5;
//        int maxIterations = 50;
//        int neq = this.enumerateDOFs();
//        double[] displacement = new double[neq];  // Initial displacement
//        double[] deltaU = new double[neq];        // Incremental displacement
//
//        ILSESolver solver = new GeneralMatrixLSESolver();
//        QuadraticMatrixInfo aInfo = solver.getAInfo();
//        IMatrix a = solver.getA();
//        aInfo.setSize(neq);
//
//        // Assemble external force vector
//        double[] fExt = new double[neq];
//        this.assembleLoadVector(fExt);
//
//        // Nonlinear Newton-Raphson loop
//        for (int iter = 0; iter < maxIterations; iter++) {
//            solver.initialize();
//            this.assembleStiffnessMatrix(a, true);  // Tangent stiffness matrix (with Geometric)
//
//            double[] fInt = new double[neq];  // Internal force vector
//            this.assembleInternalForceVector(fInt);
//
//            // Residual vector r = fExt - fInt
//            double[] residual = new double[neq];
//            for (int i = 0; i < neq; i++) {
//                residual[i] = fExt[i] - fInt[i];
//            }
//
//            // Check convergence based on residual norm
//            double normResidual = computeNorm(residual);
//            System.out.println("Iteration: " + iter + ", Residual Norm: " + normResidual);
//            if (normResidual < tolerance) {
//                System.out.println("Converged in " + iter + " iterations.");
//                break;
//            }
//
//            // Solve for incremental displacement: K_T * deltaU = residual
//            try {
//                solver.solve(residual);
//                System.arraycopy(residual, 0, deltaU, 0, neq);
//            } catch (SolveFailedException e) {
//                System.out.println("Nonlinear solve failed: " + e.getMessage());
//                return;
//            }
//
//            // Update displacements
//            for (int i = 0; i < neq; i++) {
//                displacement[i] += deltaU[i];
//            }
//
//            // Apply updated displacements
//            this.selectDisplacements(displacement);
//
//            // Update internal forces for next iteration
//            this.assembleInternalForceVector(fInt);
//        }
//    }
//
//    // Helper to calculate the norm of a vector (used for convergence check)
//    private double computeNorm(double[] vector) {
//        double norm = 0;
//        for (double v : vector) {
//            norm += v * v;
//        }
//        return Math.sqrt(norm);
//    }

    // Displacement Selection
    public void selectDisplacements(double[] solution) {
        for (Node node : nodes) {
            double[] sol = new double[3];  // 3 displacement components
            for (int i = 0; i < 3; i++) {
                if (node.getDOFNumb()[i] != -1) {
                    sol[i] = solution[node.getDOFNumb()[i]];
                } else {
                    sol[i] = 0.0;  // If DOF is fixed
                }
            }
            node.setDisplacement(sol);
        }
    }

    // Print Results
	public void printResults() {
		System.out.println("\nStructural Analysis Results:");
		System.out.println("Nodal Displacements:");
		for (Node node : nodes) {
			double[] displacements = node.getDisplacement().toArray();
			System.out.println("Node " + nodes.indexOf(node) + ": (" + displacements[0] + ", " + displacements[1] + ", " + displacements[2] + ")");
		}
		System.out.println("Element Forces:");
		for (Element element : elements) {
			double force = element.computeAxialForce();
			System.out.println("Element " + elements.indexOf(element) + ": " + force);
		}
	}

	// Thermal Analysis of Truss //
	// Assembling the global thermal stiffness matrix
	public void assembleThermalStiffnessMatrix(IMatrix kGlobal) {
	    for (Element element : elements) {
	        // Step 1: Compute the local thermal stiffness matrix for the element
	        IMatrix kThermal = element.computeThermalStiffnessMatrix();

	        // Step 2: Transform the local stiffness matrix to global coordinates
	        if (kThermal == null) {
	            System.out.println("Error: Thermal stiffness matrix for an element is null.");
	            continue;  // Skip this element if the stiffness matrix is null
	        }

	        // Step 3: Get global DOF indices for the nodes of this element (1 DOF per node)
	        Node node1 = element.getNode1();
	        Node node2 = element.getNode2();

	        // Since there is only 1 temperature DOF per node, we index each node with a single DOF.
	        int[] dofIndices = new int[] {
	            nodes.indexOf(node1),  // Temperature DOF for node 1
	            nodes.indexOf(node2)   // Temperature DOF for node 2
	        };

	        // Ensure global DOF indices are valid
	        if (dofIndices[0] >= kGlobal.getRowCount() || dofIndices[1] >= kGlobal.getRowCount()) {
	            System.out.println("Error: DOF indices exceed global stiffness matrix dimensions.");
	            continue;  // Skip this element if DOF indices are invalid
	        }

	        // Step 4: Assemble the element's stiffness matrix into the global matrix
	        for (int i = 0; i < dofIndices.length; i++) {
	            for (int j = 0; j < dofIndices.length; j++) {
	                double value = kThermal.get(i, j);
	                if (dofIndices[i] != -1 && dofIndices[j] != -1) {
	                    kGlobal.add(dofIndices[i], dofIndices[j], value);
	                }
	            }
	        }
	    }

//	    // Debugging: Print the final assembled global stiffness matrix
//	    System.out.println("\nGlobal Thermal Stiffness Matrix after assembly:");
//	    for (int i = 0; i < kGlobal.getRowCount(); i++) {
//	        for (int j = 0; j < kGlobal.getColumnCount(); j++) {
//	            System.out.print(kGlobal.get(i, j) + "\t");
//	        }
//	        System.out.println();
//	    }
	}

    // Assemble the thermal flux vector
    public void assembleThermalFluxVector(double[] rGlobal) {
        for (Element element : elements) {
            // Thermal flux of the element
            double thermalFlux = element.getThermalFlux();

            // Distribute thermal flux to connected nodes
            Node node1 = element.getNode1();
            Node node2 = element.getNode2();

            // Get global DOF indices for the nodes (one DOF per node for temperature)
            int dofIndex1 = nodes.indexOf(node1);  // Temperature DOF for node 1
            int dofIndex2 = nodes.indexOf(node2);  // Temperature DOF for node 2

            // Distribute the flux equally between the two nodes
            rGlobal[dofIndex1] += thermalFlux / 2.0;
            rGlobal[dofIndex2] += thermalFlux / 2.0;
        }

//        // Debugging: Print the final assembled thermal flux vector
//        System.out.println("Global Thermal Flux Vector after assembly:");
//        for (int i = 0; i < rGlobal.length; i++) {
//            System.out.println("rGlobal[" + i + "] = " + rGlobal[i]);
//        }
    }

    // Solve for thermal analysis using the thermal flux vector
    public void solveThermal() {
        int neq = nodes.size();  // One DOF per node for thermal analysis
        // Creates a linear system equation (LSE) solver for general matrices
        ILSESolver solver = new GeneralMatrixLSESolver();
        // Retrieves information about the matrix A (number of rows, columns, or symmetric)
		QuadraticMatrixInfo aInfo = solver.getAInfo();
		IMatrix a = solver.getA();
		double[] b = new double[neq];
		aInfo.setSize(neq);
		solver.initialize();

        // Assemble the global stiffness matrix and thermal flux vector
        this.assembleThermalStiffnessMatrix(a);
        this.assembleThermalFluxVector(b);  // Use thermal flux vector instead of load vector

        try {
            solver.solve(b);  // Saves solution in b
        } catch (SolveFailedException e) {  // Matrix singularity, numerical issues, division by zero in factorization
            System.out.println("Solve failed: " + e.getMessage());
        }

        // Store results as temperatures in nodes
        this.selectTemperatures(b);
    }

    // Modify the temperature assignment to include room temperature
    public void selectTemperatures(double[] solution) {
        for (int i = 0; i < nodes.size(); i++) {
            Node node = nodes.get(i);
            if (node.isTemperaturePrescribed()) {
                node.setTemperature(node.getPrescribedTemperature());
            } else {
                // Adjust the temperature by adding room temperature as a base
                double tempWithRoom = solution[i] + roomTemperature;
                node.setTemperature(tempWithRoom);
            }
        }
    }
    
    // Apply solar flux to elements based on their height
    public void applySolarFlux(double totalHeight, double solarFluxBase, double solarFluxTop) {
        for (Element element : elements) {
            // Calculate average height of the element (based on node positions)
            double avgHeight = (element.getNode1().getPosition().getX3() + element.getNode2().getPosition().getX3()) / 2.0;

            // Linear interpolation of solar flux based on element's height
            double solarFlux = solarFluxBase + (avgHeight / totalHeight) * (solarFluxTop - solarFluxBase);
            
            // Set the computed solar flux to the element
            element.setThermalFlux(solarFlux);
        }
    }
    
    // Print thermal results
    public void printThermalResults() {
        System.out.println("\nThermal Analysis Results:");
        for (Node node : nodes) {
            System.out.println("Node " + nodes.indexOf(node) + ": Temperature = " + node.getTemperature() + " °C");
        }
    }

    public void setRoomTemperature(double temperature) {
        roomTemperature = temperature;
    }

    public static double getRoomTemperature() {
        return roomTemperature;
    }
	
    // Thermal + Mechanical Coupling //
    // Assemble the coupling matrices (K_uθ and K_θu) separately
    public void assembleCouplingMatrix(IMatrix K_coupled, int neqU) {
        for (Element element : elements) {
            // Compute the coupling matrices (mechanical-to-thermal and thermal-to-mechanical)
            IMatrix K_uθ = element.computeThermoMechanicalCouplingMatrix();  // Mechanical-to-thermal coupling
            IMatrix K_θu = transpose(K_uθ);  // Transpose to get thermal-to-mechanical coupling

            // Get mechanical DOF indices (6 per element, 3 DOFs per node)
            int[] dofIndicesU = element.getDOFNumb();  // Mechanical DOF indices for the element

            // Get thermal DOF indices (2 DOFs, 1 per node)
            int[] dofIndicesT = new int[] {
                nodes.indexOf(element.getNode1()),  // Node 1 thermal DOF
                nodes.indexOf(element.getNode2())   // Node 2 thermal DOF
            };

            // Shift thermal DOFs by neqU to place them in the correct part of the global matrix
            int[] shiftedDofIndicesT = new int[dofIndicesT.length];
            for (int i = 0; i < dofIndicesT.length; i++) {
                shiftedDofIndicesT[i] = neqU + dofIndicesT[i];
            }

            // Assemble K_uθ (mechanical-to-thermal coupling matrix) into the top-right block of the global matrix
            assembleMatrix(K_coupled, K_uθ, dofIndicesU, shiftedDofIndicesT);

            // Assemble K_θu (thermal-to-mechanical coupling matrix) into the bottom-left block of the global matrix
            assembleMatrix(K_coupled, K_θu, shiftedDofIndicesT, dofIndicesU);

//            // Debugging: Print the global matrix after coupling assembly
//            printMatrix(K_coupled, "K_coupled (after assembling coupling matrices)");
        }
    }

    // Assemble a local matrix into a global matrix
    private void assembleMatrix(IMatrix globalMatrix, IMatrix localMatrix, int[] rowDOFIndices, int[] colDOFIndices) {
        for (int i = 0; i < rowDOFIndices.length; i++) {
            if (rowDOFIndices[i] != -1) {
                for (int j = 0; j < colDOFIndices.length; j++) {
                    if (colDOFIndices[j] != -1) {
                        globalMatrix.add(rowDOFIndices[i], colDOFIndices[j], localMatrix.get(i, j));
                    }
                }
            }
        }
    }

    // Assemble the full coupled system
    public void assembleCoupledSystem(IMatrix K_coupled, int neqU) {
        // Step 1: Assemble K_uu (mechanical stiffness) into K_coupled
        this.assembleStiffnessMatrix(K_coupled, true); // Assemble K_uu into the top-left block
        
        // Step 2: Assemble K_θθ (thermal stiffness) into K_coupled
        IMatrix K_thermal = new Array2DMatrix(nodes.size(), nodes.size());  // Separate thermal stiffness matrix
        this.assembleThermalStiffnessMatrix(K_thermal);
        // Shift the thermal DOFs by neqU to place them in the bottom-right block
        for (int i = 0; i < nodes.size(); i++) {
            for (int j = 0; j < nodes.size(); j++) {
                K_coupled.add(neqU + i, neqU + j, K_thermal.get(i, j));
            }
        }

        // Step 3: Assemble the coupling matrices (K_uθ and K_θu) separately
        this.assembleCouplingMatrix(K_coupled, neqU);

//        // Print matrices for debugging
//        printMatrix(K_coupled, "K_coupled (full matrix after assembly)");
//        printMatrix(K_thermal, "K_θθ (Thermal Stiffness)");
    }

    public void assembleThermalMechanicalLoadVector(double[] rGlobal) {
        for (Element element : elements) {
            double[] rLocal = new double[6];

            // Compute thermal-induced internal forces, passing room temperature
            double thermalStrain = element.computeThermalStrain();
            double thermalForce = element.getEModulus() * element.getArea() * thermalStrain;

            // Distribute the thermal force to nodes
            rLocal[0] = thermalForce;  // Node 1 force
            rLocal[3] = -thermalForce; // Node 2 force (opposite)

            int[] dofIndices = element.getDOFNumb();
            for (int i = 0; i < 6; i++) {
                if (dofIndices[i] != -1) {
                    rGlobal[dofIndices[i]] += rLocal[i];
                }
            }
        }
    }
    
    public void solveCoupledSystem() {
        // Number of DOFs (displacement and temperature)
        int neqU = this.enumerateDOFs();  // Number of mechanical DOFs (displacement)
        int neqT = nodes.size();          // Number of thermal DOFs (1 per node)

        // Total number of equations
        int totalDOF = neqU + neqT;

        ILSESolver solver = new GeneralMatrixLSESolver();
        QuadraticMatrixInfo aInfo = solver.getAInfo();
        IMatrix K_coupled = solver.getA();  // Full coupled matrix (mechanical + thermal)
        aInfo.setSize(totalDOF);

        // Load vectors
        double[] F_combined = new double[totalDOF];  // Combined load vector
        double[] F_mech = new double[neqU];          // Mechanical load vector
        double[] F_thermal = new double[neqT];       // Thermal load vector

        // Initialize solver
        solver.initialize();

        // Step 1: Assemble mechanical (K_uu), thermal (K_θθ), and coupling matrices (K_uθ, K_θu)
        assembleCoupledSystem(K_coupled, neqU);

        // Print matrices for debugging
        printMatrix(K_coupled, "K_coupled");

        // Step 2: Assemble the mechanical and thermal load vectors
        assembleLoadVector(F_mech);            // Mechanical loads
        assembleThermalFluxVector(F_thermal);  // Thermal flux loads

        // Print load vectors for debugging
//        printVector(F_mech, "F_mech");
//        printVector(F_thermal, "F_thermal");

        // Step 3: Combine the load vectors into F_combined
        for (int i = 0; i < neqU; i++) {
            F_combined[i] = F_mech[i];  // Fill the mechanical part
        }
        for (int i = 0; i < neqT; i++) {
            F_combined[neqU + i] = F_thermal[i];  // Fill the thermal part
        }

        // Print combined load vector
        printVector(F_combined, "F_combined");

        // Step 4: Solve the coupled system K_coupled * x_combined = F_combined
        try {
            solver.solve(F_combined);
        } catch (SolveFailedException e) {
            System.out.println("Coupled system solve failed: " + e.getMessage());
            return;
        }

        // Step 5: Extract displacements and temperatures from the solution
        double[] displacements = new double[neqU];
        double[] temperatures = new double[neqT];

        // arraycopy(source array, source postion, destination array, destination position, length)
        System.arraycopy(F_combined, 0, displacements, 0, neqU);
        System.arraycopy(F_combined, neqU, temperatures, 0, neqT);

        // Step 6: Assign results back to the structure
        selectDisplacements(displacements);
        selectTemperatures(temperatures);

        // Step 7: Print results
        printCoupledResults();
    }
    
    public void printCoupledResults() {
        System.out.println("\nThermo-Mechanical Coupling Analysis Results:");

        // Print nodal temperatures
        System.out.println("Nodal Temperatures:");
        for (Node node : nodes) {
            double temperature = node.getTemperature();
            System.out.println("Node " + nodes.indexOf(node) + ": Temperature = " + temperature + " °C");
        }

        // Print nodal displacements
        System.out.println("\nNodal Displacements:");
        for (Node node : nodes) {
            double[] displacements = node.getDisplacement().toArray();
            System.out.println("Node " + nodes.indexOf(node) + ": Displacement = (" 
                + displacements[0] + ", " + displacements[1] + ", " + displacements[2] + ") meters");
        }

        // Print element axial forces
        System.out.println("\nElement Forces (including thermal effects):");
        for (Element element : elements) {
            double axialForce = element.computeAxialForce();  // Compute axial force considering thermal strain
            System.out.println("Element " + elements.indexOf(element) + ": Axial Force = " + axialForce + " N");
        }
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

	// Print matrix values for debugging
	private void printMatrix(IMatrix matrix, String matrixName) {
	    System.out.println("Matrix: " + matrixName);
	    for (int i = 0; i < matrix.getRowCount(); i++) {
	        for (int j = 0; j < matrix.getColumnCount(); j++) {
	            System.out.print(matrix.get(i, j) + "\t");
	        }
	        System.out.println();
	    }
	    System.out.println();
	}

	// Print vector values for debugging
	private void printVector(double[] vector, String vectorName) {
	    System.out.println("Vector: " + vectorName);
	    for (int i = 0; i < vector.length; i++) {
	        System.out.println(vectorName + "[" + i + "] = " + vector[i]);
	    }
	    System.out.println();
	}
	
	public void printStructure() {
		System.out.println("There are " + getNumberOfNodes() + " nodes and " + getNumberOfElements()
				+ " elements in the structure.");
	}
}