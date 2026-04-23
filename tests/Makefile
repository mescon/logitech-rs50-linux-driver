CFLAGS = -O2 -Wall -Wextra -Wno-unused-parameter -Wno-unused-function

all: ff_matrix_test

ff_matrix_test: ff_matrix_test.c
	$(CC) $(CFLAGS) -o $@ $<

clean:
	rm -f ff_matrix_test

.PHONY: all clean
