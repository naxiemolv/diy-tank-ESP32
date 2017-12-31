#include "packageParser.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "DCMotorController.h"


void parserMoveCMD(char *cmd,int len);

int bytearraycmp(unsigned char * bytesA,char * bytesB,int len) {
    
    for (int i = 0;i<len;i++) {
        if (bytesA[i] != bytesB[i]) {
            return 0;
        }
    }
    return 1;    
}
int findCharAtIndex(unsigned char *data,int dataLen,char aChar,int count) {
    int ret = -1;
    for (int i = 0; i<dataLen ;i++) {

        unsigned char bChar = data[i];

        if (aChar == bChar) {
            count--;
            if (count == 0) {
                return i;
            }
        }
    }
    return ret;

}

void handlePackage(unsigned char *data,int len) {
    if (len>0 && data[0] == '$') {
        if (bytearraycmp(&data[1],"CMD",3) == 1) {

            int moveCMDStartIndex = findCharAtIndex(data,len,'@',2);
            int moveCMDFinishIndex = findCharAtIndex(data,len,'@',3);
            if (moveCMDStartIndex!=-1 && moveCMDFinishIndex!=-1) {
                
                char * moveCMD = (char *)&data[19];
                int moveCMDLen = moveCMDFinishIndex - moveCMDStartIndex;

                if (moveCMDLen>0) {
                    parserMoveCMD(moveCMD,moveCMDLen);                   
                }
            }

        }
    }
}

void parserMoveCMD(char *cmd,int len) {
    int start;
    int end;
    
    int ars[4] = {0,0,0,0};

    char str[10] ={}; 
    start = 0;

    for (int i = 0;i<4;i++) {
        
        if (i!=3) {
            end = findCharAtIndex((unsigned char*)cmd,len,':',i+1);
        } else {
            end = findCharAtIndex((unsigned char*)cmd,len,'@',1);
        }

        memcpy(&str[0],&cmd[start],end-start);

        str[end-start] = 0;
        int value = atoi(&str[0]);
        ars[i] = value;
        start = end+1;

    }

    updateMoveCMD(ars[0],ars[2],ars[1],ars[3]);
    
}