package model;

import fem.Constraint;
import fem.Force;
import fem.Node;
import fem.Structure;
import fem.Visualizer;
import inf.v3d.view.Viewer;

public class SmallTetraeder {
	public static Structure createStructure() {
		Structure struct = new Structure();
		double lb = 15.0;
		double r = 457.2 / 2000;  // Cross-sectional radius in meters
		double t = 10.0 / 1000;  // Wall thickness in meters
		double a = Math.PI * (Math.pow(r, 2) - Math.pow(r - t, 2));  // Cross-sectional area
		double e = 2.1e11;
        double thermalConductivity = 45.0;  // Thermal conductivity value for Steel (W/m·K)
		double alpha = 1.2e-5;  // Coefficient of thermal expansion for Steel (1/°C)
		
		Constraint c1 = new Constraint(false, false, false);
		Constraint c2 = new Constraint(true, true, false);
		Force f = new Force(0, -20e3, -100e4);
		
		// create nodes
		Node n1 = struct.addNode(0.0, 0.0, lb * Math.sqrt(2.0 / 3.0));
		Node n2 = struct.addNode(0.0, lb / Math.sqrt(3), 0);
		Node n3 = struct.addNode(-lb / 2, -lb / Math.sqrt(12.0), 0);
		Node n4 = struct.addNode(lb / 2, -lb / Math.sqrt(12.0), 0);
		// apply BCs
		n1.setForce(f);
		n2.setConstraint(c1);
		n3.setConstraint(c1);
		n4.setConstraint(c2);
		// create elements
		struct.addElement(e, a, n1, n2);
		struct.addElement(e, a, n1, n3);
		struct.addElement(e, a, n1, n4);
		struct.addElement(e, a, n2, n3);
		struct.addElement(e, a, n3, n4);
		struct.addElement(e, a, n4, n2);
		
        // Apply coefficient of thermal expansion
        for (int i = 0; i < struct.getNumberOfElements(); i++) {
            struct.getElement(i).setAlpha(alpha);
        }
        // Apply thermal conductivity
        for (int i = 0; i < struct.getNumberOfElements(); i++) {
            struct.getElement(i).setThermalConductivity(thermalConductivity);
        }
        // Assign thermal flux values to elements
        struct.getElement(1).setThermalFlux(-3);  // Element 0 has a thermal flux of 3 W/m^2
        struct.getElement(0).setThermalFlux(-8);  // Element 2 has a thermal flux of 5 W/m^2
        //return the new structure
		return struct;
	}

	public static void main(String[] args) {
		Viewer viewer = new Viewer();
		Structure struct = createStructure();
		Visualizer viz = new Visualizer(struct);
		
		// Solving the Thermo-Mechanical Coupled System
		struct.solveCoupledSystem();
		
		// Adjusting the scale
		viz.setConstraintSymbolScale(1);
		viz.setForceSymbolScale(3);
		viz.setForceSymbolRadius(0.075);
		viz.setElementForceScale(0.05);
		
		viz.drawElements(viewer);
		viz.drawConstraints(viewer);
		viz.drawElementForces(viewer);
		viz.drawDeformedStructure(viewer);
		viz.visualizeElementForces(viewer);
		viewer.setVisible(true);
	}
}
