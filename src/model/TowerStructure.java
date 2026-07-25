package model;

import fem.Constraint;
import fem.Force;
import fem.Node;
import fem.Structure;
import fem.Visualizer;
import inf.v3d.view.Viewer;

public class TowerStructure {

	    public static Structure createTowerStructure() {
	        Structure struct = new Structure();
	        double e = 2.1e11;  // Young's modulus for steel (Pa)
	        double area = 0.01;  // Reduced cross-sectional area (m^2) for more elements
	        double height = 20.0;  // Height of the tower (m)
	        double baseWidth = 7.0;  // Base width (m)
	        double topWidth = 3.0;  // Top width (m)
	        double solarFluxBase = 10.0;  // Solar flux density at the base (kW/m^2)
	        double solarFluxTop = 14.0;   // Solar flux density at the top (kW/m^2)
	        
	        // Base constraints (all DOFs fixed)
	        Constraint fixedConstraint = new Constraint(false, false, false);

	        // Create base nodes
	        Node n1 = struct.addNode(baseWidth / 2, baseWidth / 2, 0);
	        Node n2 = struct.addNode(-baseWidth / 2, baseWidth / 2, 0);
	        Node n3 = struct.addNode(baseWidth / 2, -baseWidth / 2, 0);
	        Node n4 = struct.addNode(-baseWidth / 2, -baseWidth / 2, 0);

	        // Apply fixed constraints to base nodes
	        n1.setConstraint(fixedConstraint);
	        n2.setConstraint(fixedConstraint);
	        n3.setConstraint(fixedConstraint);
	        n4.setConstraint(fixedConstraint);

	        // Create middle nodes (mid-height)
	        Node n5 = struct.addNode(topWidth / 2, topWidth / 2, height / 3);
	        Node n6 = struct.addNode(-topWidth / 2, topWidth / 2, height / 3);
	        Node n7 = struct.addNode(topWidth / 2, -topWidth / 2, height / 3);
	        Node n8 = struct.addNode(-topWidth / 2, -topWidth / 2, height / 3);

	        // Create second level nodes (2/3 height)
	        Node n9 = struct.addNode(topWidth / 2, topWidth / 2, 2 * height / 3);
	        Node n10 = struct.addNode(-topWidth / 2, topWidth / 2, 2 * height / 3);
	        Node n11 = struct.addNode(topWidth / 2, -topWidth / 2, 2 * height / 3);
	        Node n12 = struct.addNode(-topWidth / 2, -topWidth / 2, 2 * height / 3);

	        // Create top node
	        Node nTop = struct.addNode(0, 0, height);

	        // Vertical elements connecting base to first level
	        struct.addElement(e, area, n1, n5);
	        struct.addElement(e, area, n2, n6);
	        struct.addElement(e, area, n3, n7);
	        struct.addElement(e, area, n4, n8);

	        // Vertical elements connecting first level to second level
	        struct.addElement(e, area, n5, n9);
	        struct.addElement(e, area, n6, n10);
	        struct.addElement(e, area, n7, n11);
	        struct.addElement(e, area, n8, n12);

	        // Vertical elements connecting second level to top
	        struct.addElement(e, area, n9, nTop);
	        struct.addElement(e, area, n10, nTop);
	        struct.addElement(e, area, n11, nTop);
	        struct.addElement(e, area, n12, nTop);

	        // Add cross-bracing elements at each level for stability
	        struct.addElement(e, area, n5, n6);  // Bracing between mid-height nodes
	        struct.addElement(e, area, n6, n7);
	        struct.addElement(e, area, n7, n8);
	        struct.addElement(e, area, n8, n5);
	        struct.addElement(e, area, n5, n7);
	        struct.addElement(e, area, n8, n6);

	        struct.addElement(e, area, n9, n10);  // Bracing between second-level nodes
	        struct.addElement(e, area, n10, n11);
	        struct.addElement(e, area, n11, n12);
	        struct.addElement(e, area, n12, n9);
	        struct.addElement(e, area, n9, n11);
	        struct.addElement(e, area, n12, n10);

	        // Add diagonal bracing between base and top for added stiffness
	        struct.addElement(e, area, n1, n10);
	        struct.addElement(e, area, n1, n11);
	        struct.addElement(e, area, n2, n9);
	        struct.addElement(e, area, n2, n12);
	        struct.addElement(e, area, n3, n12);
	        struct.addElement(e, area, n3, n9);
	        struct.addElement(e, area, n4, n10);
	        struct.addElement(e, area, n4, n11);

	        // Apply a wind load force (in x-direction, simplified wind load)
	        Force windForce = new Force(-1000, 0, -5000);  // Wind and downward force
	        nTop.setForce(windForce);
	        n5.setForce(windForce);
	        n6.setForce(windForce);
	        n7.setForce(windForce);
	        n8.setForce(windForce);
	        n9.setForce(windForce);
	        n10.setForce(windForce);
	        n11.setForce(windForce);
	        n12.setForce(windForce);
	        
	        // Apply solar flux based on height (simplified as linear distribution)
	        struct.applySolarFlux(height, solarFluxBase, solarFluxTop);

	        return struct;
	    }

	    public static void main(String[] args) {
	        Viewer viewer = new Viewer();
	        Structure struct = createTowerStructure();
	        Visualizer viz = new Visualizer(struct);

	        // Solve mechanical problem
	        struct.solve(true);  // Solve with geometric stiffness
	        struct.printResults();  // Print mechanical results

	        // Solve thermal problem
	        struct.solveThermal();  // Separate method for solving thermal analysis
	        struct.printThermalResults();  // Print thermal results

	        // Visualize the tower structure and forces
			viz.setConstraintSymbolScale(0.8);
			viz.setForceSymbolScale(1.5);
			viz.setForceSymbolRadius(0.05);
			viz.setElementForceScale(2);
			
	        viz.drawElements(viewer);
	        viz.drawConstraints(viewer);  // Show constraints
	        viz.drawElementForces(viewer);  // Show forces
	        viz.drawDeformedStructure(viewer);  // Deformed structure visualization
//			viz.visualizeElementForces(viewer);
	        viewer.setVisible(true);
	    }
	}