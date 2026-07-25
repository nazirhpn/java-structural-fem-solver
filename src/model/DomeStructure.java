package model;

import fem.*;
import inf.v3d.view.Viewer;

public class DomeStructure {

    public static Structure createDomeStructure() {
        Structure struct = new Structure();

        double radius = 10.0;  // Radius of the dome
        int numLevels = 5;     // Number of vertical levels in the dome
        int numNodesPerLevel = 8;  // Number of nodes per level (circle)

        // Create dome nodes in circular pattern (polar coordinates)
        for (int i = 0; i < numLevels; i++) {
            double z = radius * (1 - (double) i / numLevels);  // Height of the node
            double r = Math.sqrt(radius * radius - z * z);    // Radius of the circle at this level
            for (int j = 0; j < numNodesPerLevel; j++) {
                double theta = 2 * Math.PI * j / numNodesPerLevel;  // Angle around the circle
                double x = r * Math.cos(theta);
                double y = r * Math.sin(theta);
                struct.addNode(x, y, z);
            }
        }

        // Fix the base level nodes (bottom) to prevent any movement
        int totalCount = struct.getNumberOfNodes();
        for (int i = totalCount - 1; i > totalCount - numNodesPerLevel - 1; i--) {
            Node baseNode = struct.getNode(i);  // First level nodes (bottom base nodes)
            baseNode.setConstraint(new Constraint(false, false, false));  // Fully constrained
        }

        // Connect nodes with truss elements (triangular connections and cross-bracing)
        for (int i = 0; i < numLevels - 1; i++) {
            int baseIndex = i * numNodesPerLevel;
            int nextIndex = (i + 1) * numNodesPerLevel;

            for (int j = 0; j < numNodesPerLevel; j++) {
                int nextJ = (j + 1) % numNodesPerLevel;

                // Connect to the next level (vertical elements)
                struct.addElement(210e9, 0.01, struct.getNode(baseIndex + j), struct.getNode(nextIndex + j));
                struct.addElement(210e9, 0.01, struct.getNode(baseIndex + j), struct.getNode(nextIndex + nextJ));
                
                // Connect to the same level (horizontal elements)
                struct.addElement(210e9, 0.01, struct.getNode(baseIndex + j), struct.getNode(baseIndex + nextJ));

                // Add diagonal cross-bracing between levels
                struct.addElement(210e9, 0.01, struct.getNode(baseIndex + j), struct.getNode(nextIndex + nextJ));
                struct.addElement(210e9, 0.01, struct.getNode(baseIndex + nextJ), struct.getNode(nextIndex + j));
            }
        }

        // Apply forces (gravity, wind, and rain forces) to the top nodes
        Force gravity = new Force(0, 0, -9.81e2);  // Gravity force
        Force wind = new Force(1e2, 0, 0);        // Wind force
        Force rain = new Force(0, 0, -5e2);       // Rain force

        // Apply forces to the top nodes and upper levels
        int numNodesTop = totalCount - numNodesPerLevel;  // The last level + top node is the upper part
        for (int i = 0; i < numNodesTop; i++) {  // Apply forces to the topmost nodes
            struct.getNode(i).setForce(gravity);
            struct.getNode(i).setForce(wind);
            struct.getNode(i).setForce(rain);
        }
        
        return struct;
    }

    public static void main(String[] args) {
        Viewer viewer = new Viewer();
        Structure struct = createDomeStructure();
        Visualizer viz = new Visualizer(struct);

        // Solve mechanical problem
        struct.solve(false);
        struct.printResults();

        // Visualization
		viz.setConstraintSymbolScale(0.8);
		viz.setForceSymbolScale(1.5);
		viz.setForceSymbolRadius(0.05);
		viz.setElementForceScale(2);
		
        viz.drawElements(viewer);
        viz.drawConstraints(viewer);  // Show constraints
        viz.drawElementForces(viewer);  // Show forces
        viz.drawDeformedStructure(viewer);  // Deformed structure visualization
        viz.drawDisplacementsPolygon(viewer);  // Draws a displacement gradient towards the undeformed shape
		viz.visualizeElementForces(viewer);
        viewer.setVisible(true);
    }
}
