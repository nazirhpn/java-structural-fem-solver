package fem;

import inf.v3d.obj.CylinderSet;
import inf.v3d.obj.PolygonSet;
import inf.v3d.obj.Arrow;
import iceb.jnumerics.Vector3D;
import inf.v3d.view.Viewer;

public class Visualizer {
	static double displacementScale;
	private double symbolScale;
	private Structure Structure;
    private double constraintSymbolScale = 1.0;  // Scale for constraint symbols
    private double forceSymbolScale = 3;         // Scale for force arrows
    private double forceSymbolRadius = 0.075;    // Radius of the force arrows
    private double elementForceScale = 1.0;      // Scale for visualizing element forces
    
	public Visualizer(Structure struct){
		this.Structure = struct;
	}

    public void drawElements(Viewer viewer) {
        CylinderSet cylinders = new CylinderSet();
        for (int i = 0; i < Structure.getNumberOfElements(); i++) {
            Element element = Structure.getElement(i);
            Node node1 = element.getNode1();
            Node node2 = element.getNode2();
            double radius = Math.sqrt(element.getArea() / Math.PI);
            cylinders.addCylinder(node1.getPosition().toArray(), node2.getPosition().toArray(), radius);
        }
        viewer.addObject3D(cylinders);
        // cylinders.setColor("lightGray");
    }

    // The arrows indicate directions where the node is constrained
    public void drawConstraints(Viewer viewer) {
        for (int i = 0; i < Structure.getNumberOfNodes(); i++) {
            Node node = Structure.getNode(i);
            Constraint constraint = node.getConstraint();
            if (constraint != null) {
                double[] position = node.getPosition().toArray();

                // For each direction (x, y, z), if the node is constrained, draw an arrow.
                for (int j = 0; j < 3; j++) {
                    if (!constraint.isFree(j)) {
                        double[] endPosition = position.clone();
                        endPosition[j] += constraintSymbolScale;  // Apply constraint symbol scale

                        Arrow arrow = new Arrow(endPosition, position);
                        arrow.setRadius(constraintSymbolScale * 0.2);
                        viewer.addObject3D(arrow);
                    }
                }
            }
        }
    }

    public void drawElementForces(Viewer viewer) {
        for (int i = 0; i < Structure.getNumberOfNodes(); i++) {
            Node node = Structure.getNode(i);
            Force force = node.getForce();
            if (force != null) {
                double[] position = node.getPosition().toArray();
                
                // For each component of the force (x, y, z), draw an arrow in the appropriate direction
                for (int j = 0; j < 3; j++) {
                    double f = force.getComponents(j);
                    if (f != 0) {
                        double[] endPosition = position.clone();
                        endPosition[j] += forceSymbolScale;  // Apply force symbol scale

                        Arrow arrow = new Arrow(endPosition, position);
                        arrow.setRadius(forceSymbolRadius);  // Apply force symbol radius
                        arrow.setColor("magenta");
                        viewer.addObject3D(arrow);
                    }
                }
            }
        }
    }

    public void drawDisplacementsPolygon(Viewer viewer) {
        for (int i = 0; i < Structure.getNumberOfElements(); i++) {
            Element element = Structure.getElement(i);
            PolygonSet ps = new PolygonSet();
            displacementScale = 10000;
            
            // Calculate the displaced position of the first node of the element.
            Vector3D originalPosition1 = element.getNode1().getPosition();
            Vector3D displacements1 = element.getNode1().getDisplacement();
            double x1 = originalPosition1.getX1() + displacementScale * displacements1.getX1();
            double y1 = originalPosition1.getX2() + displacementScale * displacements1.getX2();
            double z1 = originalPosition1.getX3() + displacementScale * displacements1.getX3();

            // Calculate the displaced position of the second node of the element.
            Vector3D originalPosition2 = element.getNode2().getPosition();
            Vector3D displacements2 = element.getNode2().getDisplacement();
            double x2 = originalPosition2.getX1() + displacementScale * displacements2.getX1();
            double y2 = originalPosition2.getX2() + displacementScale * displacements2.getX2();
            double z2 = originalPosition2.getX3() + displacementScale * displacements2.getX3();

            // Insert vertices for the polygon based on original and displaced positions.
            ps.insertVertex(x1, y1, z1, 1);
            ps.insertVertex(originalPosition1.toArray(), 2);
            ps.insertVertex(x2, y2, z2, 3);
            ps.insertVertex(originalPosition2.toArray(), 4);
            ps.polygonComplete();  // Complete the polygon.
            ps.setVisible(true);
            ps.setColoringByData(true);    // Color the polygon based on data (e.g., displacement).
            ps.setOutlinesVisible(true);
            ps.setContourLinesVisible(true);
            ps.createColors();
            viewer.addObject3D(ps);
        }
    }

    public void drawDeformedStructure(Viewer viewer) {
        CylinderSet cylinders = new CylinderSet();
        for (int i = 0; i < Structure.getNumberOfElements(); i++) {
            Element element = Structure.getElement(i);
            displacementScale = 10000;  // Magnification factor for deformations.

            // Compute the displaced positions of the element's nodes.
            Vector3D originalPosition1 = element.getNode1().getPosition();
            Vector3D displacements1 = element.getNode1().getDisplacement();
            double x1 = originalPosition1.getX1() + displacementScale * displacements1.getX1();
            double y1 = originalPosition1.getX2() + displacementScale * displacements1.getX2();
            double z1 = originalPosition1.getX3() + displacementScale * displacements1.getX3();
            double[] point1 = {x1, y1, z1};

            Vector3D originalPosition2 = element.getNode2().getPosition();
            Vector3D displacements2 = element.getNode2().getDisplacement();
            double x2 = originalPosition2.getX1() + displacementScale * displacements2.getX1();
            double y2 = originalPosition2.getX2() + displacementScale * displacements2.getX2();
            double z2 = originalPosition2.getX3() + displacementScale * displacements2.getX3();
            double[] point2 = {x2, y2, z2};

            // Draw the deformed element as a cylinder.
            double radius = Math.sqrt(element.getArea() / Math.PI);
            cylinders.addCylinder(point1, point2, radius);
        }
        viewer.addObject3D(cylinders);
        cylinders.setColor("orange");  // Set the color of deformed elements.
    }

    // Visualize element forces, applying element force scale
    public void visualizeElementForces(Viewer viewer) {
        PolygonSet polygon_set = new PolygonSet();
        for (int i = 0; i < Structure.getNumberOfElements(); i++) {
            Element element = Structure.getElement(i);
            Vector3D X1 = element.getDisplacementOfNode1();
            Vector3D X2 = element.getDisplacementOfNode2();
            
            // Compute the direction vector d
            Vector3D d = (X2.subtract(X1)).normalize();
            // Compute two normal vectors n1 and n2
            Vector3D n1 = new Vector3D(1, 1, 1);  // Arbitrary choice
            Vector3D n2 = (d.vectorProduct(n1)).normalize();
            Vector3D p = (n1.vectorProduct(d)).normalize();
            
            if (Math.abs(d.scalarProduct(n1)) > 0.9) {
                p = (n2.vectorProduct(d)).normalize();
            }
            
            double N = element.computeAxialForce();
            symbolScale = elementForceScale * 0.00003;  // Apply element force scale
            Vector3D s1 = X1.add(p.multiply(N * symbolScale));
            Vector3D s2 = X2.add(p.multiply(N * symbolScale));
            
            polygon_set.insertVertex(X1.getX1(), X1.getX2(), X1.getX3(), N);
            polygon_set.insertVertex(s1.getX1(), s1.getX2(), s1.getX3(), N);
            polygon_set.insertVertex(s2.getX1(), s2.getX2(), s2.getX3(), N);
            polygon_set.insertVertex(X2.getX1(), X2.getX2(), X2.getX3(), N);
            polygon_set.polygonComplete();
            polygon_set.setVisible(true);
            polygon_set.setColoringByData(true);
            polygon_set.createColors();
        }
        viewer.addObject3D(polygon_set);
    }
    
    // Methods to set the scales
    public void setConstraintSymbolScale(double scale) {
        this.constraintSymbolScale = scale;
    }

    public void setForceSymbolScale(double scale) {
        this.forceSymbolScale = scale;
    }

    public void setForceSymbolRadius(double radius) {
        this.forceSymbolRadius = radius;
    }
   
    public void setElementForceScale(double scale) {
        this.elementForceScale = scale;
    }
}