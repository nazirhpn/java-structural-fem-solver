package model;

import fem.*;
import inf.v3d.view.Viewer;

public class LatticeCube {

    public static Structure createLatticeStructure() {
        Structure struct = new Structure();
        double e = 2.1e11;  // Young's modulus for steel (Pa)
        double area = 0.01;  // Cross-sectional area (m^2)
        double cellSize = 1.0;  // Size of each lattice cell (m)
        int gridSize = 3;  // 3x3x3 lattice (gridSize+1 nodes in each direction)

        // Constraints
        Constraint fixedConstraint = new Constraint(false, false, false);  // Fix the bottom nodes

        // Add nodes and connect them in a cubic lattice
        Node[][][] nodes = new Node[gridSize + 1][gridSize + 1][gridSize + 1];

        // Create nodes for the lattice
        for (int i = 0; i <= gridSize; i++) {
            for (int j = 0; j <= gridSize; j++) {
                for (int k = 0; k <= gridSize; k++) {
                    nodes[i][j][k] = struct.addNode(i * cellSize, j * cellSize, k * cellSize);

                    // Fix bottom layer of nodes
                    if (k == 0) {
                        nodes[i][j][k].setConstraint(fixedConstraint);
                    }

                    // Apply a downward force on the top layer
                    if (k == gridSize) {
                        nodes[i][j][k].setForce(new Force(0, 0, -10000));  // Downward force of 10kN
                    }

                    // Connect nodes in X, Y, Z directions to form basic grid elements
                    if (i > 0) struct.addElement(e, area, nodes[i - 1][j][k], nodes[i][j][k]);  // X-direction
                    if (j > 0) struct.addElement(e, area, nodes[i][j - 1][k], nodes[i][j][k]);  // Y-direction
                    if (k > 0) struct.addElement(e, area, nodes[i][j][k - 1], nodes[i][j][k]);  // Z-direction
                }
            }
        }

        // Adding cross-bracing to increase structural stability (inspired by the dome structure)
        for (int i = 0; i < gridSize; i++) {
            for (int j = 0; j < gridSize; j++) {
                for (int k = 0; k < gridSize; k++) {
                    // Cross-bracing in XY plane
                    struct.addElement(e, area, nodes[i][j][k], nodes[i + 1][j + 1][k + 1]);
                    struct.addElement(e, area, nodes[i + 1][j][k], nodes[i][j + 1][k + 1]);

                    // Cross-bracing in YZ plane
                    struct.addElement(e, area, nodes[i][j][k], nodes[i + 1][j + 1][k + 1]);
                    struct.addElement(e, area, nodes[i][j + 1][k], nodes[i + 1][j][k + 1]);

                    // Cross-bracing in XZ plane
                    struct.addElement(e, area, nodes[i][j][k], nodes[i + 1][j + 1][k + 1]);
                    struct.addElement(e, area, nodes[i + 1][j][k], nodes[i + 1][j][k + 1]);
                }
            }
        }

        return struct;
    }

    public static void main(String[] args) {
        Viewer viewer = new Viewer();
        Structure struct = createLatticeStructure();
        Visualizer viz = new Visualizer(struct);

        // Solve the structural problem
        struct.solve(false);  // No geometric stiffness in this simplified test
        struct.printResults();  // Print mechanical results

        // Visualization
        viz.setConstraintSymbolScale(0.3);
        viz.setForceSymbolScale(1);
        viz.setForceSymbolRadius(0.03);
        viz.setElementForceScale(0.5);

        viz.drawElements(viewer);
        viz.drawConstraints(viewer);
        viz.drawElementForces(viewer);
        viz.drawDeformedStructure(viewer);
		viz.visualizeElementForces(viewer);
        viewer.setVisible(true);
    }
}