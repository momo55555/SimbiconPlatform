#pragma once

#include <stdarg.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#pragma warning (push)
#pragma warning (disable : 4275 4251)
#include <vector>
#pragma warning (pop)


#define DynamicArray std::vector
typedef unsigned int uint;

/**
	This method reads all the doubles from the given file and stores them in the array of doubles that is passed in
*/
inline void readDoublesFromFile(FILE* f, DynamicArray<double> *d){
	double temp;
	while (fscanf_s(f, "%lf\n", &temp) == 1)
		d->push_back(temp);
}


/**
	This method returns a pointer to the first non-white space character location in the provided buffer
*/
inline char* lTrim(char* buffer){
	while (*buffer==' ' || *buffer=='\t' || *buffer=='\n' || *buffer=='\r')
		buffer++;
	return buffer;
}

inline char* rTrim(char* buffer){
	int index = (int)strlen(buffer) - 1;
	while (index>=0){
		if (buffer[index]==' ' || buffer[index]=='\t' || buffer[index]=='\n' || buffer[index]=='\r'){
			buffer[index] = '\0';
			index--;
		}
		else
			break;
	}
	return buffer;
}

inline char* trim(char* buffer){
	return rTrim(lTrim(buffer));
}

/**
	This method reads a line from a file. It does not return empty lines or ones that start with a pound key - those are assumed to be comments.
	This method returns true if a line is read, false otherwise (for instance the end of file is met).
*/
inline bool readValidLine(char* line, FILE* fp){
	while (!feof(fp)){
		fgets(line, 100, fp);
		char* tmp = trim(line);
		if (tmp[0]!='#' && tmp[0]!='\0')
			return true;
	}

	return false;
}

/**
	This method returns a DynamicArray of char pointers that correspond to the addressed
	of the tokens that are separated by white space in the string that is passed in as a pointer.
*/
inline DynamicArray<char*> getTokens(char* input){
	DynamicArray<char*> result;
	input = lTrim(input);
	//read in the strings one by one - assume that each tokens are less than 100 chars in length
	while (input[0]!='\0'){
		result.push_back(input);
		char tempStr[100];
		sscanf_s(input, "%s", tempStr);
		input = lTrim(input + strlen(tempStr));
	}
	return result;
}

#define __max__(x,y) (((x)>(y))?(x):(y))

