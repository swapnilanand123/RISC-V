# RISCV_Single_Cycle_Core

A RISC-V (Reduced Instruction Set Computing - Five) single-cycle core refers to a processor core design that executes each instruction in a single clock cycle. RISC-V is an open-source instruction set architecture (ISA) that is gaining popularity due to its simplicity, modularity, and the fact that it is freely available for anyone to use and implement.

In a single-cycle core design, each stage of the instruction pipeline, including instruction fetch, decode, execute, memory access, and write-back, is completed in one clock cycle. This results in a straightforward and predictable execution model, but it may limit the clock speed and overall performance of the processor, especially as the complexity of the instruction set or the desired functionality increases.

A RISC-V single-cycle core would typically implement a simplified version of the RISC-V ISA and have a dedicated hardware unit for each stage of the instruction pipeline. The simplicity of the design makes it suitable for educational purposes and for understanding the basic principles of processor architecture. However, in real-world applications, most modern processors use pipelining and multiple cycles to execute instructions efficiently, allowing for higher clock speeds and better overall performance.

Implementing a RISC-V processor as a single-cycle core is a good starting point for learning about computer architecture and digital design. As students progress, they may explore more advanced concepts such as pipelining, superscalar architectures, and out-of-order execution to design processors with improved performance.

# Architecture
![image](https://github.com/swapnilanand123/RISCV_Single_Cycle_Core/assets/143795450/3f8e61bc-0e89-4b5a-b58e-159af0b4c0ef)

# Stages	of	Exection	on Datapath
![image](https://github.com/swapnilanand123/RISCV_Single_Cycle_Core/assets/143795450/ded8fe12-7968-43b1-822e-e3b4dd5a935c)

# Waveform
![image](https://github.com/swapnilanand123/RISCV_Single_Cycle_Core/assets/143795450/da8cb198-d6f8-44e2-a839-81c752d61091)


