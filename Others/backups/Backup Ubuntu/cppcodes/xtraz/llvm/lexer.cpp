#include <iostream>
#include <fstream>
#include <string>
using namespace std;

enum Token {
  tok_eof = -1,

  // commands
  tok_def = -2,
  tok_extern = -3,

  // primary
  tok_identifier = -4,
  tok_number = -5,
};

static std::string IdentifierStr; // Filled in if tok_identifier
static double NumVal;             // Filled in if tok_number

class lexer
{
public:
	void tokenize(string line)
	{
		cout << "\n|" << line << "|" << endl;
		for (int i = 0; i < line.length();) {
			static int LastChar = ' ';
			while (LastChar == ' ' || LastChar == '\n' || (int)LastChar == 0)//(isspace(LastChar))	// skip whitespace
    			LastChar = line[i++];
    		if (true)//(isalpha(LastChar)) { // identifier: [a-zA-Z][a-zA-Z0-9]*
				IdentifierStr = LastChar;
				while (isalnum((LastChar = line[i++])))
					IdentifierStr += LastChar;
				if (IdentifierStr == "def")
					cout << "tok_def : <" << IdentifierStr << "> " << IdentifierStr.length() << endl;
					//return tok_def;
				else if (IdentifierStr == "extern")
					cout << "tok_extern : <" << IdentifierStr << "> " << IdentifierStr.length() << endl;
					//return tok_extern;
				else
					cout << "tok_identifier : <" << IdentifierStr << "> " <<  IdentifierStr.length() << endl;
					//return tok_identifier;
				//for (int j = 0; j < IdentifierStr.length(); j++)
					//cout << (int)IdentifierStr[j] << endl;
		}
	}
};

	int main(int argc, char** argv)
	{
		//lexer lex;
		//pdd.cage();
		//return 1; 
		// reading a text file
		lexer lx;
		string line;
		ifstream src (argv[1]);
		if (src.is_open()) {
			while (getline(src,line)) {
		  		lx.tokenize(line);
			}
			src.close();
		}
		else cout << "Unable to open file"; 
		cout << endl;
		
		return 0;
	}