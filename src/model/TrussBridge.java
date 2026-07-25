package model;

import fem.*;
import inf.v3d.view.Viewer;

public class TrussBridge {

    public static Structure createTrussBridgeStructure() {
        Structure struct = new Structure();
        double e = 2.1e11;  // Young's modulus for steel (Pa)
        double area = 0.1;  // Cross-sectional area (m^2)
        int numSegmentsTop = 6;  // Number of segments along the top
        int numSegments = 8;  // Number of segments along the bottom length of the bridge
        double topLength = 60.0;  // Total length of the bridge (m)
        double bridgeLength = 80.0;  // Total length of the bridge (m)
        double bridgeHeight = 10.0;   // Height between the top and bottom chords
        double deckWidth = 10.0;      // Width of the bridge deck
        double bottomChordSpacing = deckWidth / 3;  // Space between bottom chords

        // Step size for each segment along the length of the bridge
        double segmentLength = bridgeLength / numSegments;

        // Constraints
        Constraint fixedConstraint = new Constraint(false, false, false);  // Fixed constraint for base nodes

        // Create bottom chord nodes (lower part of the bridge)
        Node[][] bottomChords = new Node[4][numSegments + 1];
        for (int i = 0; i <= numSegments; i++) {
            double x = -bridgeLength / 2 + i * segmentLength;

            // Create 4 equally spaced bottom chords
            bottomChords[0][i] = struct.addNode(x, -deckWidth / 2, 0);
            bottomChords[1][i] = struct.addNode(x, -bottomChordSpacing / 2, 0);
            bottomChords[2][i] = struct.addNode(x, bottomChordSpacing / 2, 0);
            bottomChords[3][i] = struct.addNode(x, deckWidth / 2, 0);
        }

        // Create top chord nodes (upper part of the bridge)
        Node[][] topChords = new Node[2][numSegmentsTop + 1];
        for (int i = 0; i <= numSegmentsTop; i++) {
            double x = -topLength / 2 + i * segmentLength;

            // Two top chords spaced equally
            topChords[0][i] = struct.addNode(x, -deckWidth / 2, bridgeHeight);
            topChords[1][i] = struct.addNode(x, deckWidth / 2, bridgeHeight);
        }

        // Connect adjacent bottom chord nodes (horizontal elements)
        for (int i = 0; i < numSegments; i++) {
            // Horizontal connections for all four bottom chords
            for (int j = 0; j < 4; j++) {
                struct.addElement(e, area, bottomChords[j][i], bottomChords[j][i + 1]);  // Bottom chords
            }
        }
        
        // Horizontal bracings between bottom chords
        for (int i = 0; i <= numSegments; i++) {
        struct.addElement(e, area, bottomChords[0][i], bottomChords[3][i]);
        struct.addElement(e, area, bottomChords[1][i], bottomChords[2][i]);
        struct.addElement(e, area, bottomChords[2][i], bottomChords[3][i]);
    }

        // Connect adjacent top chord nodes (horizontal elements)
        for (int i = 0; i < numSegmentsTop; i++) {
            struct.addElement(e, area, topChords[0][i], topChords[0][i + 1]);  // Left top chord
            struct.addElement(e, area, topChords[1][i], topChords[1][i + 1]);  // Right top chord

            // Reduce diagonal bracing at the supports
            if (i <= numSegmentsTop) {
                // Diagonal bracing between top chords except near supports
                struct.addElement(e, area, topChords[0][i], topChords[1][i + 1]);  // Diagonal bracing left to right
                struct.addElement(e, area, topChords[1][i], topChords[0][i + 1]);  // Diagonal bracing right to left
            }
            // Horizontal bracing for all segments
            struct.addElement(e, area, topChords[0][i], topChords[1][i]);  // Horizontal bracing
        }
        
        // Add vertical members between top and bottom chords
        for (int i = 0; i <= numSegmentsTop; i++) {
            // Connect outer top chords to outer bottom chords only
            struct.addElement(e, area, topChords[0][i], bottomChords[0][i+1]);  // Left outer
            struct.addElement(e, area, topChords[1][i], bottomChords[3][i+1]);  // Right outer

            // Diagonal cross-bracing between outer chords only
            struct.addElement(e, area, bottomChords[0][i], topChords[0][i]);  // Diagonal left to right
            struct.addElement(e, area, bottomChords[3][i], topChords[1][i]);
            struct.addElement(e, area, bottomChords[0][i+2], topChords[0][i]);  // Diagonal right to left
            struct.addElement(e, area, bottomChords[3][i+2], topChords[1][i]);
        }
        
        // Additional cross-bracing
        struct.addElement(e, area, topChords[0][numSegmentsTop], topChords[1][numSegmentsTop]);
        struct.addElement(e, area, topChords[1][0], bottomChords[0][0]);
        struct.addElement(e, area, topChords[0][0], bottomChords[3][0]);
        struct.addElement(e, area, topChords[0][numSegmentsTop], bottomChords[3][numSegments]);
        struct.addElement(e, area, topChords[1][numSegmentsTop], bottomChords[0][numSegments]);

        // Fix the base of the support pillars
        bottomChords[0][0].setConstraint(fixedConstraint);
        bottomChords[1][0].setConstraint(fixedConstraint);
        bottomChords[2][0].setConstraint(fixedConstraint);
        bottomChords[3][0].setConstraint(fixedConstraint);
        bottomChords[0][numSegments].setConstraint(fixedConstraint);
        bottomChords[1][numSegments].setConstraint(fixedConstraint);
        bottomChords[2][numSegments].setConstraint(fixedConstraint);
        bottomChords[3][numSegments].setConstraint(fixedConstraint);

        // Apply gravity force on the bridge
        Force gravity = new Force(0, 0, -9.81);  // Gravity force
        for (int i = 0; i < struct.getNumberOfNodes(); i++) {
        	struct.getNode(i).setForce(gravity);
        }

        // Apply vehicle load as point forces on the middle of the bridge
        int midSegment = numSegments / 2;
        Force vehicleLoad = new Force(0, 0, -5e3);  // Vehicle load (downward force)
        for (int j = 0; j < 4; j++) {
            bottomChords[j][midSegment].setForce(vehicleLoad);  // Apply vehicle load at the middle
        }

        // Apply wind load on the top chords
        Force windLoad = new Force(500, 0, 0);  // Wind force (horizontal)
        for (int i = 0; i <= numSegmentsTop; i++) {
            topChords[0][i].setForce(windLoad);    // Wind acting on left top chord nodes
            topChords[1][i].setForce(windLoad);    // Wind acting on right top chord nodes
        }

        return struct;
    }

    public static void main(String[] args) {
        Viewer viewer = new Viewer();
        Structure struct = createTrussBridgeStructure();
        Visualizer viz = new Visualizer(struct);

        // Solve structural problem
        struct.solve(false);  // No geometric stiffness in this simplified test
        struct.printResults();  // Print mechanical results

        // Visualization
        viz.setConstraintSymbolScale(2);
        viz.setForceSymbolScale(4);
        viz.setForceSymbolRadius(0.15);
        viz.setElementForceScale(2);

        viz.drawElements(viewer);
        viz.drawConstraints(viewer);
        viz.drawElementForces(viewer);
//        viz.drawDeformedStructure(viewer);
        viewer.setVisible(true);
    }
}
