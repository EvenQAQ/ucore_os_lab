# Ucore Lab1 Report

## 练习1

#### 1、操作系统镜像文件ucore.img是如何一步一步生成的？(需要比较详细地解释Makefile中每一条相关命令和命令参数的含义，以及说明命令导致的结果)

首先生成kernel：

```makefile
$(kernel): tools/kernel.ld

$(kernel): $(KOBJS)
	@echo + ld $@
	$(V)$(LD) $(LDFLAGS) -T tools/kernel.ld -o $@ $(KOBJS)
	@$(OBJDUMP) -S $@ > $(call asmfile,kernel)
	@$(OBJDUMP) -t $@ | $(SED) '1,/SYMBOL TABLE/d; s/ .* / /; /^$$/d' > $(call symfile,kernel)
```

生成kernel需要kernel.ld init.o readline.o stdio.o kdebug.o kmonitor.o panic.o clock.o console.o intr.o picirq.o trap.o trapentry.o vectors.o pmm.o string.o

Kernel.ld已经存在，生成上述.o文件的代码为：

```makefile
$(call add_files_cc,$(call listf_cc,$(KSRCDIR)),kernel,$(KCFLAGS))
```

然后是生成bootblock：

```makefile
$(bootblock): $(call toobj,$(bootfiles)) | $(call totarget,sign)
	@echo + ld $@
	$(V)$(LD) $(LDFLAGS) -N -e start -Ttext 0x7C00 $^ -o $(call toobj,bootblock)
	@$(OBJDUMP) -S $(call objfile,bootblock) > $(call asmfile,bootblock)
	@$(OBJCOPY) -S -O binary $(call objfile,bootblock) $(call outfile,bootblock)
	@$(call totarget,sign) $(call outfile,bootblock) $(bootblock)
```

由上述代码可以看出，bootblock由bootasm.o, bootmain.o, sign生成

生成bootasm.o和bootmain.o的代码：

```makefile
bootfiles = $(call listf_cc,boot)
$(foreach f,$(bootfiles),$(call cc_compile,$(f),$(CC),$(CFLAGS) -Os -nostdinc))
```

生成sign工具的代码:

```makefile
$(call add_files_host,tools/sign.c,sign,sign)
$(call create_target_host,sign,sign)
```

最后由kernel和bootblock生成ucore.img的代码：

```makefile
$(UCOREIMG): $(kernel) $(bootblock)
	$(V)dd if=/dev/zero of=$@ count=10000
	$(V)dd if=$(bootblock) of=$@ conv=notrunc
	$(V)dd if=$(kernel) of=$@ seek=1 conv=notrunc
```

 #### 2、一个被系统认为是符合规范的硬盘主引导扇区的特征是什么?

由sign.c可知，一个磁盘主引导扇区有512字节，且导数两个字节为0x55和0xAA

```C
char buf[512];
...
buf[510] = 0x55;
buf[511] = 0xAA;
```

## 练习二

#### 1、从CPU加电后执行的第一条指令开始，单步跟踪BIOS的执行。

参照附录中单部跟踪的方法

1 修改 lab1/tools/gdbinit,内容为:

```
set architecture i8086
target remote :1234
```

2 在 lab1目录下，执行

```
make debug
```

3 在看到gdb的调试界面(gdb)后，在gdb调试界面下执行如下命令

```
si
```

即可单步跟踪BIOS了。

4 在gdb界面下，可通过如下命令来看BIOS的代码

```
 x /2i $pc  //显示当前eip处的汇编指令
```

#### 2、在初始化位置0x7c00 设置实地址断点,测试断点正常。

```
Breakpoint 2, 0x00007c00 in ?? ()
(gdb) x /2i $pc
=> 0x7c00:      cli
   0x7c01:      cld
   0x7c02:      xor    %eax,%eax
   0x7c04:      mov    %eax,%ds
   0x7c06:      mov    %eax,%es
   0x7c08:      mov    %eax,%ss
   0x7c0a:      in     $0x64,%al
   0x7c0c:      test   $0x2,%al
   0x7c0e:      jne    0x7c0a
   0x7c10:      mov    $0xd1,%al
   0x7c12:      out    %al,$0x64
   0x7c14:      in     $0x64,%al
```

#### 3、从0x7c00开始跟踪代码运行,将单步跟踪反汇编得到的代码与bootasm.S和 bootblock.asm进行比较。

```
b *0x7c00
c
x /10i $pc
```

查看代码

```
0x7c00:      cli
0x7c01:      cld
0x7c02:      xor    %eax,%eax
0x7c04:      mov    %eax,%ds
0x7c06:      mov    %eax,%es
0x7c08:      mov    %eax,%ss
0x7c0a:      in     $0x64,%al
```

与bootasm.S和bootblock.asm一致。

#### 4、自己找一个bootloader或内核中的代码位置，设置断点并进行测试。

设置断点0x7d0d

```
b *0x7d0d
c
x /10i $pc
```

输出结果

```
0x7d0d:      push   %ebp
0x7d0e:      xor    %ecx,%ecx
0x7d10:      mov    $0x1000,%edx
0x7d15:      mov    $0x10000,%eax
0x7d1a:      mov    %esp,%ebp
0x7d1c:      push   %esi
0x7d1d:      push   %ebx
```

## 练习三

#### 分析bootloader进入保护模式的过程。

1、初始化，置零

```
# start address should be 0:7c00, in real mode, the beginning address of the running bootloader
.globl start
start:
.code16                                             # Assemble for 16-bit mode
    cli                                             # Disable interrupts
    cld                                             # String operations increment

    # Set up the important data segment registers (DS, ES, SS).
    xorw %ax, %ax                                   # Segment number zero
    movw %ax, %ds                                   # -> Data Segment
    movw %ax, %es                                   # -> Extra Segment
    movw %ax, %ss                                   # -> Stack Segment
```

2、开启A20地址栈

```
    # Enable A20:
    #  For backwards compatibility with the earliest PCs, physical
    #  address line 20 is tied low, so that addresses higher than
    #  1MB wrap around to zero by default. This code undoes this.
seta20.1:
    inb $0x64, %al                                  # Wait for not busy(8042 input buffer empty).
    testb $0x2, %al
    jnz seta20.1

    movb $0xd1, %al                                 # 0xd1 -> port 0x64
    outb %al, $0x64                                 # 0xd1 means: write data to 8042's P2 port

seta20.2:
    inb $0x64, %al                                  # Wait for not busy(8042 input buffer empty).
    testb $0x2, %al
    jnz seta20.2

    movb $0xdf, %al                                 # 0xdf -> port 0x60
    outb %al, $0x60                                 # 0xdf = 11011111, means set P2's A20 bit(the 1 bit) to 1
```

3、初始化GDT，开启保护模式

```
    # Switch from real to protected mode, using a bootstrap GDT
    # and segment translation that makes virtual addresses
    # identical to physical addresses, so that the
    # effective memory map does not change during the switch.
    lgdt gdtdesc
    movl %cr0, %eax
    orl $CR0_PE_ON, %eax
    movl %eax, %cr0

    # Jump to next instruction, but in 32-bit code segment.
    # Switches processor into 32-bit mode.
    ljmp $PROT_MODE_CSEG, $protcseg
```

4、跳转基地址

```
    # Jump to next instruction, but in 32-bit code segment.
    # Switches processor into 32-bit mode.
    ljmp $PROT_MODE_CSEG, $protcseg
```

5、设置段寄存器，并开启bootmain

```
.code32                                             # Assemble for 32-bit mode
protcseg:
    # Set up the protected-mode data segment registers
    movw $PROT_MODE_DSEG, %ax                       # Our data segment selector
    movw %ax, %ds                                   # -> DS: Data Segment
    movw %ax, %es                                   # -> ES: Extra Segment
    movw %ax, %fs                                   # -> FS
    movw %ax, %gs                                   # -> GS
    movw %ax, %ss                                   # -> SS: Stack Segment

    # Set up the stack pointer and call into C. The stack region is from 0--start(0x7c00)
    movl $0x0, %ebp
    movl $start, %esp
    call bootmain
```

## 练习四

### 分析bootloader加载ELF格式的OS的过程

1、readsect:从secno读取数据到dst

```
/* readsect - read a single sector at @secno into @dst */
static void
readsect(void *dst, uint32_t secno) {
    // wait for disk to be ready
    waitdisk();

    outb(0x1F2, 1);                         // count = 1
    outb(0x1F3, secno & 0xFF);
    outb(0x1F4, (secno >> 8) & 0xFF);
    outb(0x1F5, (secno >> 16) & 0xFF);
    outb(0x1F6, ((secno >> 24) & 0xF) | 0xE0);
    outb(0x1F7, 0x20);                      // cmd 0x20 - read sectors

    // wait for disk to be ready
    waitdisk();

    // read a sector
    insl(0x1F0, dst, SECTSIZE / 4);
}
```

2、readseg: 读取一定长度的内容

```
/* *
 * readseg - read @count bytes at @offset from kernel into virtual address @va,
 * might copy more than asked.
 * */
static void
readseg(uintptr_t va, uint32_t count, uint32_t offset) {
    uintptr_t end_va = va + count;

    // round down to sector boundary
    va -= offset % SECTSIZE;

    // translate from bytes to sectors; kernel starts at sector 1
    uint32_t secno = (offset / SECTSIZE) + 1;

    // If this is too slow, we could read lots of sectors at a time.
    // We'd write more to memory than asked, but it doesn't matter --
    // we load in increasing order.
    for (; va < end_va; va += SECTSIZE, secno ++) {
        readsect((void *)va, secno);
    }
}
```

3、bootmain: 加载ELF

```
/* bootmain - the entry of bootloader */
void
bootmain(void) {
    // read the 1st page off disk
    readseg((uintptr_t)ELFHDR, SECTSIZE * 8, 0);

    // is this a valid ELF?
    if (ELFHDR->e_magic != ELF_MAGIC) {
        goto bad;
    }

    struct proghdr *ph, *eph;

    // load each program segment (ignores ph flags)
    ph = (struct proghdr *)((uintptr_t)ELFHDR + ELFHDR->e_phoff);
    eph = ph + ELFHDR->e_phnum;
    for (; ph < eph; ph ++) {
        readseg(ph->p_va & 0xFFFFFF, ph->p_memsz, ph->p_offset);
    }

    // call the entry point from the ELF header
    // note: does not return
    ((void (*)(void))(ELFHDR->e_entry & 0xFFFFFF))();
```

## 练习五

### 实现函数调用堆栈跟踪函数

按照注释填写代码内容，打印堆栈状态

```C
void
print_stackframe(void) {
	uint32_t ebp = read_ebp();
	uint32_t eip = read_eip();
	int i;
	uint32_t *p;
	for (i = 0; i < STACKFRAME_DEPTH && ebp; i++)
	{
		cprintf("ebp:%08x eip:%08x ", ebp, eip);
		p = (uint32_t*)ebp;
		cprintf("args:%08x %08x %08x %08x ", p[2], p[3], p[4], p[5]);
		cprintf("\n");
		print_debuginfo(eip-1);
		eip = p[1];
		ebp = p[0];
	}
     /* LAB1 2016011354 : STEP 1 */
     /* (1) call read_ebp() to get the value of ebp. the type is (uint32_t);
      * (2) call read_eip() to get the value of eip. the type is (uint32_t);
      * (3) from 0 .. STACKFRAME_DEPTH
      *    (3.1) printf value of ebp, eip
      *    (3.2) (uint32_t)calling arguments [0..4] = the contents in address (unit32_t)ebp +2 [0..4]
      *    (3.3) cprintf("\n");
      *    (3.4) call print_debuginfo(eip-1) to print the C calling function name and line number, etc.
      *    (3.5) popup a calling stackframe
      *           NOTICE: the calling funciton's return addr eip  = ss:[ebp+4]
      *                   the calling funciton's ebp = ss:[ebp]
      */
	
}
```

## 练习六

### 完善中断初始化和处理

#### 中断向量表中一个表项占多少字节？其中哪几位代表中断处理代码的入口？

中断向量表一个表项占用8字节，其中2-3字节是段选择子，0-1字节和6-7字节拼成位移，
两者联合便是中断处理程序的入口地址。

#### 请编程完善kern/trap/trap.c中对中断向量表进行初始化的函数idt_init。

初始化中断向量表，特判0x80

```C
/* idt_init - initialize IDT to each of the entry points in kern/trap/vectors.S */
void
idt_init(void) {
	extern uintptr_t __vectors[];
	int i;
	for (i = 0; i < 256; i++)
	{
		if (i == 0x80)
		{
			SETGATE(idt[i], 1, GD_KTEXT, __vectors[i], 3)
		}
		else
		{
			SETGATE(idt[i], 0, GD_KTEXT, __vectors[i], 0)
		}
	}
	lidt(&idt_pd);
     /* LAB1 2016011354 : STEP 2 */
     /* (1) Where are the entry addrs of each Interrupt Service Routine (ISR)?
      *     All ISR's entry addrs are stored in __vectors. where is uintptr_t __vectors[] ?
      *     __vectors[] is in kern/trap/vector.S which is produced by tools/vector.c
      *     (try "make" command in lab1, then you will find vector.S in kern/trap DIR)
      *     You can use  "extern uintptr_t __vectors[];" to define this extern variable which will be used later.
      * (2) Now you should setup the entries of ISR in Interrupt Description Table (IDT).
      *     Can you see idt[256] in this file? Yes, it's IDT! you can use SETGATE macro to setup each item of IDT
      * (3) After setup the contents of IDT, you will let CPU know where is the IDT by using 'lidt' instruction.
      *     You don't know the meaning of this instruction? just google it! and check the libs/x86.h to know more.
      *     Notice: the argument of lidt is idt_pd. try to find it!
      */
	
}
```

#### 请编程完善trap.c中的中断处理函数trap，在对时钟中断进行处理的部分填写trap函数

刚开始使用了在时钟中断增加计时器count的方法，ticks是后来阅读代码发现的。

```C
//static int count = 0;

case IRQ_OFFSET + IRQ_TIMER:
	//count++;
	ticks++;
	if (ticks % TICK_NUM == 0)
	{
		print_ticks();
		//count = 0;
	}
    break;
    /* LAB1 YOUR CODE : STEP 3 */
    /* handle the timer interrupt */
    /* (1) After a timer interrupt, you should record this event using a global variable (increase it), such as ticks in kern/driver/clock.c
     * (2) Every TICK_NUM cycle, you can print some info using a funciton, such as print_ticks().
     * (3) Too Simple? Yes, I think so!
     */
	
```

make qemu-nox的结果

```
+ cc kern/trap/trap.c
+ ld bin/kernel
记录了10000+0 的读入
记录了10000+0 的写出
5120000 bytes (5.1 MB, 4.9 MiB) copied, 0.0161118 s, 318 MB/s
记录了1+0 的读入
记录了1+0 的写出
512 bytes copied, 9.3176e-05 s, 5.5 MB/s
记录了146+1 的读入
记录了146+1 的写出
74828 bytes (75 kB, 73 KiB) copied, 0.000259574 s, 288 MB/s
WARNING: Image format was not specified for 'bin/ucore.img' and probing guessed raw.
         Automatically detecting the format is dangerous for raw images, write operations on block 0 will be restricted.
         Specify the 'raw' format explicitly to remove the restrictions.
(THU.CST) os is loading ...

Special kernel symbols:
  entry  0x00100000 (phys)
  etext  0x00103474 (phys)
  edata  0x0010ea16 (phys)
  end    0x0010fd20 (phys)
Kernel executable memory footprint: 64KB
ebp:00007b38 eip:00100a3d args:00010094 00010094 00007b68 0010007f 
    kern/debug/kdebug.c:306: print_stackframe+22
ebp:00007b48 eip:00100d3a args:00000000 00000000 00000000 00007bb8 
    kern/debug/kmonitor.c:125: mon_backtrace+10
ebp:00007b68 eip:0010007f args:00000000 00007b90 ffff0000 00007b94 
    kern/init/init.c:48: grade_backtrace2+19
ebp:00007b88 eip:001000a1 args:00000000 ffff0000 00007bb4 00000029 
    kern/init/init.c:53: grade_backtrace1+27
ebp:00007ba8 eip:001000be args:00000000 00100000 ffff0000 00100043 
    kern/init/init.c:58: grade_backtrace0+19
ebp:00007bc8 eip:001000df args:00000000 00000000 00000000 00103480 
    kern/init/init.c:63: grade_backtrace+26
ebp:00007be8 eip:00100050 args:00000000 00000000 00000000 00007c4f 
    kern/init/init.c:28: kern_init+79
ebp:00007bf8 eip:00007d6e args:c031fcfa c08ed88e 64e4d08e fa7502a8 
    <unknow>: -- 0x00007d6d --
++ setup timer interrupts
100 ticks
100 ticks
100 ticks
100 ticks
100 ticks
100 ticks
100 ticks
100 ticks
```

