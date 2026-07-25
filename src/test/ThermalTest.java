package test;

import fem.Constraint;
import fem.Force;
import fem.Node;
import fem.Structure;
import fem.Visualizer;
import inf.v3d.view.Viewer;

public class ThermalTest {
    
    public static Structure createStructure() {
        Structure struct = new Structure();
        double lb = 1.0;
        double r = 0.4572 / 2;  // Cross-sectional radius in meters
        double t = 0.01;  // Wall thickness in meters
        double a = Math.PI * (Math.pow(r, 2) - Math.pow(r - t, 2));  // Cross-sectional area
        double e = 2.1e11;  // Young's modulus for Steel (Pa)
        double thermalConductivity = 45.0;  // Thermal conductivity value for Steel (W/m·K)

        // Define mechanical boundary conditions and forces
        Constraint c1 = new Constraint(false, false, false);  // Allow displacements
        Constraint c2 = new Constraint(true, true, false);    // Fix node 4 in x and y directions
        Force f = new Force(0, -20e2, -100e3);  // Mechanical force on node 1

        // Create nodes
        Node n1 = struct.addNode(0.0, 0.0, lb * Math.sqrt(2.0 / 3.0));
        Node n2 = struct.addNode(0.0, lb / Math.sqrt(3), 0);
        Node n3 = struct.addNode(-lb / 2, -lb / Math.sqrt(12.0), 0);
        Node n4 = struct.addNode(lb / 2, -lb / Math.sqrt(12.0), 0);

        // Apply mechanical boundary conditions
        n1.setForce(f);
        n2.setConstraint(c1);
        n3.setConstraint(c1);
        n4.setConstraint(c2);

        // Create elements with mechanical and thermal properties
        struct.addElement(e, a, n1, n2).setThermalConductivity(thermalConductivity);
        struct.addElement(e, a, n1, n3).setThermalConductivity(thermalConductivity);
        struct.addElement(e, a, n1, n4).setThermalConductivity(thermalConductivity);
        struct.addElement(e, a, n2, n3).setThermalConductivity(thermalConductivity);
        struct.addElement(e, a, n3, n4).setThermalConductivity(thermalConductivity);
        struct.addElement(e, a, n4, n2).setThermalConductivity(thermalConductivity);
        
        // Assign thermal flux values to elements
        struct.getElement(0).setThermalFlux(3);  // Element 0 has a thermal flux of 3 W/m^2
        struct.getElement(2).setThermalFlux(5);  // Element 2 has a thermal flux of 5 W/m^2
        struct.getElement(4).setThermalFlux(3);  // Element 4 has a thermal flux of 3 W/m^2

        // Set room temperature
        struct.setRoomTemperature(20);  // Room temperature is set to 20°C

        return struct;
    }

    public static void main(String[] args) {
        Viewer viewer = new Viewer();
        Structure struct = createStructure();
        Visualizer viz = new Visualizer(struct);

        // Solve the mechanical problem first
        System.out.println("=== Solving Mechanical Problem ===");
        struct.solve(true);  // Solve with geometric stiffness (true for geometric nonlinearity)
        struct.printResults();  // Print mechanical results

        // Solve the thermal problem
        System.out.println("\n=== Solving Thermal Problem ===");
        struct.solveThermal();  // Solve for thermal analysis
        struct.printThermalResults();  // Print thermal results
        
        // Visualization settings
        viz.setConstraintSymbolScale(0.2);
        viz.setForceSymbolScale(0.5);
        viz.setForceSymbolRadius(0.02);
        viz.setElementForceScale(0.2);

        // Visualization of the structure
        viz.drawElements(viewer);
        viz.drawConstraints(viewer);
        viz.drawElementForces(viewer);
        viz.drawDeformedStructure(viewer);
        viz.visualizeElementForces(viewer);
        viewer.setVisible(true);
    }
}
