#include<createoi.h>
#include<stdio.h>


int main(){
	int a;
	startOI_MT("/dev/ttyUSB0");
	while(1){
		a = runCoverAndDockDemo();
		printf("a = %d\n", a);
	}	
	stopOI_MT();
	return 0;
}
