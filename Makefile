LDLIBS=-lm -lglfw -lepoxy -lGLU -lmmal -lbcm_host -lvcos -lmmal_core -lmmal_util -lmmal_vc_client
LDFLAGS=-L/opt/vc/lib
CFLAGS=-I/opt/vc/include
CPPFLAGS=-g -O1

%.o : %.c
	$(CC) $(CFLAGS) -c -o $@ $<

all: raspiraw

raspiraw: raspiraw.o shader_utils.o 
	$(CC) $(LDFLAGS) -o $@ $^ $(LDLIBS)

clean:
	-rm -f *.o
	-rm -f raspiraw

