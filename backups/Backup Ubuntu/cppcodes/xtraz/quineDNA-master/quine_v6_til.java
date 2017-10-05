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
	final static int gen = 0;
	final static int orgmID = 1;						// Organism ID	odd - female (initial zygote), even - male
	final static int typeID = 0;						// 0 - Zygote, 1 - Brain, 2 - Body, 3 - Gamate
	static int size[][] = {{0,2,2,2},{0,0,0,0},{0,0,0,0},{2,0,0,0}};	// Cells generated per type
	static int organism[][];	// Cells generated per type
	static int eyeColr[];
	final static String genotype = "1010010200000000100000000000000000000000FFFFFFFF0100FF00FFFFFFFF";
	final static String pheno = "A08B0C830C0D80D0D875082E0870F0C81038A0AA";	// Phenotype that will define organism features - each DNA ladder has two paired AGCT, thus 2 radix-4 number, or 1 Hexdecimal number
	final static String maleInit =  "219A83C8D888F8FVC8V8D81A91ABDF83FE028F65";
	public static void main(String args[])
	{
		// Test
		decode();
		System.exit(1);


		try {
			ProcessBuilder pb = new ProcessBuilder("rm","quine_v6_G"+gen+"_O"+orgmID+"T"+typeID+".java");	// Kill source cell
			Process p = pb.start();
			p.waitFor();
		}
		catch (Exception e) {
		}
		String malePheno = "";
		if(typeID == 3) {
			if (orgmID%2 == 0) {	// if male zygote, return phenotype for crossover by female
				System.out.println(pheno);
				return;
			}
			else {
				malePheno = args[0];
			}
		}
		
		// Decode DNA


		BufferedWriter bw = null;
		for(int type=0;type<size[typeID].length;type++) {
			for (int nos=0;nos<size[typeID][type];nos++) {
				int oi = (orgmID-1)*size[typeID][type]+nos+1;		// Organism ID of new generation
		   		String cn = type != 3 ? "" : (oi%2 == 0 ? "M" : "F");	// Class Name : genderless/female/male
		   		int gi = type == 0 ? (gen+1) : gen;			// Generation ID of new generation
		   		String op = oi == 1 ? pheno : maleInit;			// Offspring Phenotype is otherwise : typeID == 3 ? cross(malePheno,oi) : pheno;
		   		String DNA = "import java.io.*;\nclass quine"+cn+"\n{\n\tfinal static int gen = "+gi+";\n\tfinal static int orgmID = "+oi+";\n\tfinal static int typeID = "+type+";\n\tfinal static int size[][] = {{0,1,1,1},{0,0,0,0},{0,0,0,0},{2,0,0,0}};\n\tfinal static String pheno = \""+op+"\";\n\tpublic static void main(String args[])\n\t{\n\t\ttry {\n\t\t\tProcessBuilder pb = new ProcessBuilder(\"rm\",\"quine_v6_G\"+gen+\"_O\"+orgmID+\"T\"+typeID+\".java\");\n\t\t\tProcess p = pb.start();\n\t\t\tp.waitFor();\n\t\t}\n\t\tcatch (Exception e) {\n\t\t}\n\t\tString malePheno = \"\";\n\t\tif(typeID == 3) {\n\t\t\tif (orgmID%2 == 0) {\n\t\t\t\tSystem.out.println(pheno);\n\t\t\t\treturn;\n\t\t\t}\n\t\t\telse {\n\t\t\t\tmalePheno = args[0];\n\t\t\t}\n\t\t}\n\t\tBufferedWriter bw = null;\n\t\tfor(int type=0;type<size[typeID].length;type++) {\n\t\t\tfor (int nos=0;nos<size[typeID][type];nos++) {\n\t\t\t\tint oi = (orgmID-1)*size[typeID][type]+nos+1;\n\t\t\t\tString cn = type != 3 ? \"\" : (oi%2 == 0 ? \"M\" : \"F\");\n\t\t\t\tint gi = type == 0 ? (gen+1) : gen;\n\t\t\t\tString op = typeID == 3 ? cross(malePheno,oi) : pheno;\n\t\t\t\tString DNA = @;\n\t\t\t\ttry {\n\t\t\t\t\tbw = new BufferedWriter(new FileWriter(\"quine_v6_G\"+(type == 0 ? (gen+1) : gen)+\"_O\"+((orgmID-1)*size[typeID][type]+nos+1)+\"T\"+type+\".java\"));\n\t\t\t\t\tbw.write(process(DNA,cn,gi,oi,type,op));\n\t\t\t\t\tbw.flush();\n\t\t\t\t\tbw.close();\n\t\t\t\t}\n\t\t\t\tcatch (Exception e) {\n\t\t\t\t}\n\t\t\t}\n\t\t}\n\t\t\n\t}\n\tpublic static String process(String baseDNA,String cn,int gi,int oi,int type,String op)\n\t{\n\t\tString bs = baseDNA.replaceAll(\"\\\\\\\\\", \"\\\\\\\\\\\\\\\\\");\n\t\tString bn = bs.replaceAll(\"\\n\", \"\\\\\\\\n\");\n\t\tString bt = bn.replaceAll(\"\\t\", \"\\\\\\\\t\");\n\t\tString dq = bt.replaceAll(\"\\\"\", \"\\\\\\\\\\\"\");\n\t\tString cl = dq.replaceFirst(\"quine\"+cn, \"quine\\\"+cn+\\\"\");\n\t\tString g = cl.replaceFirst(\"gen = \"+gi, \"gen = \\\"+gi+\\\"\");\n\t\tString c = g.replaceFirst(\"orgmID = \"+oi, \"orgmID = \\\"+oi+\\\"\");\n\t\tString t = c.replaceFirst(\"typeID = \"+type, \"typeID = \\\"+type+\\\"\");\n\t\tString cr = t.replaceFirst(op, \"\\\"+op+\\\"\");\n\t\tString newDNA = \"\";\n\t\tfor (int i=0;i<baseDNA.length();i++)\n\t\t\tif (baseDNA.charAt(i) == 64)\n\t\t\t\tnewDNA = baseDNA.substring(0,i)+\"\\\"\"+cr+\"\\\"\"+baseDNA.substring(i+1);\n\t\treturn newDNA;\n\t}\n\tpublic static String cross(String phenoMate,int oi)\n\t{\n\t\tint cp = (int)(Math.random()*pheno.length());\n\t\tif (oi%2 == 0)\n\t\t\treturn pheno.substring(0,cp)+phenoMate.substring(cp);\n\t\telse\n\t\t\treturn phenoMate.substring(0,cp)+pheno.substring(cp);\n\t}\n}";
				try {	
					bw = new BufferedWriter(new FileWriter("quine_v6_G"+(type == 0 ? (gen+1) : gen)+"_O"+((orgmID-1)*size[typeID][type]+nos+1)+"T"+type+".java"));
		   			bw.write(process(DNA,cn,gi,oi,type,op));
		   			bw.flush();
					bw.close();
		   		}
		   		catch (Exception e) {
				}
			}
		}	
	}
	private static String process(String baseDNA,String cn,int gi,int oi,int type,String op)
	{
		String bs = baseDNA.replaceAll("\\\\", "\\\\\\\\");
		String bn = bs.replaceAll("\n", "\\\\n");
		String bt = bn.replaceAll("\t", "\\\\t");
		String dq = bt.replaceAll("\"", "\\\\\"");
		String cl = dq.replaceFirst("quine"+cn, "quine\"+cn+\"");
		String g = cl.replaceFirst("gen = "+gi, "gen = \"+gi+\"");
		String c = g.replaceFirst("orgmID = "+oi, "orgmID = \"+oi+\"");
		String t = c.replaceFirst("typeID = "+type, "typeID = \"+type+\"");
		String cr = t.replaceFirst(op, "\"+op+\"");
		String newDNA = "";
		for (int i=0;i<baseDNA.length();i++)
			if (baseDNA.charAt(i) == 64)
				newDNA = baseDNA.substring(0,i)+"\""+cr+"\""+baseDNA.substring(i+1);
		return newDNA;
	}
	private static String cross(String phenoMate,int oi)
	{
		int cp = (int)(Math.random()*pheno.length()); 	// Crossover point
		if (oi%2 == 0)
			return pheno.substring(0,cp)+phenoMate.substring(cp);
		else
			return phenoMate.substring(0,cp)+pheno.substring(cp);
	}
	private static void decode()
	{
		// DNA = 1-010010200000000 1-000000000000000 00-000000FFFFFFFF 01-00FF00FFFFFFFF
		int cno = 0, tt, p, c, v, pc; // Chromosome ID, Tissue Types, Parent Cell, Child Cell, Value, Primary Colour
		String chr = ""; 
		for(int i = 0; i < genotype.length(); i += 16) {	// Packet Size = 16 Hex symbols
			if (Integer.parseInt(genotype.substring(i,i+1)) == 1) {
				chr = chr + genotype.substring(i+1,i+16);
			}
			else {
				chr = chr + genotype.substring(i+2,i+16);
				switch(cno) {
					case 0 :	// size[][]
						tt = (int)Math.floor(Math.sqrt(chr.length()));
						organism = new int[tt][tt];
						for (p = 0; p < tt; p++) {
							for (c = 0; c < tt; c++) {
								v = (int)chr.charAt(p*tt+c)-48;
								if (v > 9)
									v -= 7;
								organism[p][c] = v;
							}
						}
						// 0 - Zygotes, 1 - F Gamates, 2 - M Gamates, 3 - Neurons, 4 - Eyes, 5 - RBCs
						/*
						0 --> 0,1,0,0,1,0
						1 --> 2,0,0,0,0,0
						2 --> 0,0,0,0,0,0
						3 --> 0,0,0,0,0,0
						4 --> 0,0,0,0,0,0
						5 --> 0,0,0,0,0,0
						*/
						// Chr = 010010200000000:000000000000000:000000
						// 1-010010200000000 1-000000000000000 00-000000FFFFFFFF
						break;
					case 1 :	// eye colour
						eyeColr = new int[3];
						for (pc = 0; pc < 3; pc++) { // 3 primary colour
							v = (int)chr.charAt(pc*2)-48;
							if (v > 9)
								v -= 7;
							eyeColr[pc] = v*16;		// 8-bit colour
							v = (int)chr.charAt(pc*2+1)-48;
							if (v > 9)
								v -= 7;
							eyeColr[pc] += v;
							// rr - Red, gg - Green, bb - Blue
							/*
							Green Eyes: 0,255,0 --> Chr = 00FF00
							// 01-00FF00FFFFFFFF
							*/
						}
						break;
					case 2 :	// neural network for strategic learning
						
						break;
				}
				chr = "";
				cno++;
			}
		}
		/*
		System.out.println("Eye Colour : ("+eyeColr[0]+","+eyeColr[1]+","+eyeColr[2]+")");
		for (p = 0; p < organism[0].length; p++) {
			for (c = 0; c < organism[0].length; c++) {
				System.out.print(organism[p][c]+" ");
			}
			System.out.println();
		}
		*/
	}
}
