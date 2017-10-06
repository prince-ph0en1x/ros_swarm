/*

Description:

 This code emulates binary cell division of an organism with 1 zygote, 1 body, 1 brain and 1 gamate cell.

Other improvements:

 * Rule to run : Each generated java file should be run only once per simulation
 * v6_runLiv.sh
 * Set simulation generation in script file

 * mutation of phenotype
 * mutation of size
 * more meaningful variable naming
 * implement v6_runLivDie.sh
 * CRC of string

*/

import java.io.*;
class quine		// Initial female zygote that creates two individuals of opposite sex by asexual binary division
{
	public static void main(String args[])
	{
		String s = "F";
		System.out.println((int)s.charAt(0)-48-7);
	}
}
