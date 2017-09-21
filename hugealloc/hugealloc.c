#include <fcntl.h>
#include <sys/mman.h>
#include <errno.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/ioctl.h>

#define LENGTH 128*1024*1024
#define PS_ADDR 0

int main(){
	int fd, fd_kix, retval;
	void *addr;

	fd = open("/mnt/huge/kix", O_CREAT | O_RDWR);
	if(fd < 0){
		perror("HUGEPAGE: Unable to create file in /mnt/huge");
		return -1;
	}

	addr = mmap(0, LENGTH, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
	if(addr == MAP_FAILED){
		perror("HUGEPAGE: Unable to map into hugepage");
		close(fd);
		unlink("/mnt/huge/kix");
		return -1;
	}

	fd_kix = open("/dev/kix", O_RDWR);
	if(fd_kix < 0){
		perror("kix_file: Unable to open kix dev file");
		return -1;
	}

	retval = ioctl(fd_kix, PS_ADDR, (unsigned long)addr);
	if(retval == -1){
		perror("kix_file ioctl: Unable to send kix the mem_addr");
	}

	return 0; 
}