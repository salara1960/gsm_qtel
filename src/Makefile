NAME=writer

$(NAME): func.o restore.o
	${CROSS_COMPILE}gcc -o $(NAME) func.o restore.o -I ${SYSROOT}/usr/include
	${CROSS_COMPILE}strip $(NAME)
func.o: func.c
	${CROSS_COMPILE}gcc -c func.c -O2 -Wall -D_GNU_SOURCE  -I ${SYSROOT}/usr/include
restore.o: restore.c
	${CROSS_COMPILE}gcc -c restore.c -O2 -Wall -D_GNU_SOURCE  -I ${SYSROOT}/usr/include
clean:
	rm -f *.o $(NAME)



