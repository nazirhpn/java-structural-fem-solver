package test ;

import fem . Structure ;
import inf . text . ArrayFormat ;
//import test.SetupTest ;

public class EnumerateTest2 {
	public static void main ( String [] args ) {
		Structure struct = model.SmallTetraeder.createStructure ();
// solve
		struct.solve(true);
		struct.printResults();
// print equation numbers
		System . out . println ( " Node degrees of freedom " );
		for ( int i = 0; i < struct . getNumberOfNodes (); i ++) {
			int [] dofNumbers = struct . getNode (i ). getDOFNumb ();
			System . out . println ( ArrayFormat . format ( dofNumbers ));
		}
		System . out . println ( " Element degrees of freedom " );
		for ( int i = 0; i < struct . getNumberOfElements (); i ++) {
			int [] dofNumbers = struct . getElement (i ). getDOFNumb ();
			System . out . println ( ArrayFormat . format ( dofNumbers ));
		}
	}
}