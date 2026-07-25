package test;
import iceb.jnumerics.*;
import iceb.jnumerics.lse.*;
import inf.text.ArrayFormat;
public class SetupTest2 {
	public static void main(String[] args) {		
		int neq = 4;
		ILSESolver solver = new GeneralMatrixLSESolver();
		QuadraticMatrixInfo aInfo = solver.getAInfo();
		IMatrix a = solver.getA();
		double[] b = new double[neq];
		aInfo.setSize(neq);
		solver.initialize();
		
		a.set(0, 0, 5);
		b[0] = 8;
		
		for (int i = 1; i < b.length; i++) {a.set(0, i, 1);a.set(i, 0, 1);a.set(i, i, 1);b[i] = 2;}
		System.out.println(" Solving A x = b");
		System.out.println(" Matrix A");
		System.out.println(MatrixFormat.format(a));
		System.out.println(" Vector b");
		System.out.println(ArrayFormat.format(b));
		
		try {solver.solve(b);} catch (SolveFailedException e) {System.out.println(" Solve failed : " + e.getMessage());}
		
		System.out.println(" Solution x");
		System.out.println(ArrayFormat.format(b));
		
//		Viewer viewer = new Viewer();
//		Sphere m = new Sphere(0, 0, 0);
//		m.setColor("red");
//		m.setRadius(0.1);
//		viewer.addObject3D(m);
//		viewer.setVisible(true);
	}
}
