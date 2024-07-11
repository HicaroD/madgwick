build-shared-lib:
	gcc -O3 -shared -o madgwick.so -fPIC madgwick.c
