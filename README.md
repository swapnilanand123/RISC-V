# RISCV_Single_Cycle_Core

A RISC-V (Reduced Instruction Set Computing - Five) single-cycle core refers to a processor core design that executes each instruction in a single clock cycle. RISC-V is an open-source instruction set architecture (ISA) that is gaining popularity due to its simplicity, modularity, and the fact that it is freely available for anyone to use and implement.

In a single-cycle core design, each stage of the instruction pipeline, including instruction fetch, decode, execute, memory access, and write-back, is completed in one clock cycle. This results in a straightforward and predictable execution model, but it may limit the clock speed and overall performance of the processor, especially as the complexity of the instruction set or the desired functionality increases.

A RISC-V single-cycle core would typically implement a simplified version of the RISC-V ISA and have a dedicated hardware unit for each stage of the instruction pipeline. The simplicity of the design makes it suitable for educational purposes and for understanding the basic principles of processor architecture. However, in real-world applications, most modern processors use pipelining and multiple cycles to execute instructions efficiently, allowing for higher clock speeds and better overall performance.

Implementing a RISC-V processor as a single-cycle core is a good starting point for learning about computer architecture and digital design. As students progress, they may explore more advanced concepts such as pipelining, superscalar architectures, and out-of-order execution to design processors with improved performance.

# Architecture
![image](https://github.com/swapnilanand123/RISC-V/assets/143795450/4adce460-1662-4a68-9d8f-9e420dbf4906)


# Stages	of	Exection	on Datapath
![image](https://github.com/swapnilanand123/RISCV_Single_Cycle_Core/assets/143795450/ded8fe12-7968-43b1-822e-e3b4dd5a935c)

# Waveform
![image](https://github.com/swapnilanand123/RISCV_Single_Cycle_Core/assets/143795450/da8cb198-d6f8-44e2-a839-81c752d61091)

# RISC-V Pipeline Core
A RISC-V Pipeline Core refers to a processor core design that uses a pipeline architecture to execute instructions. Unlike a single-cycle core where each instruction completes in one clock cycle, a pipeline core breaks down the instruction execution into multiple stages, allowing multiple instructions to be in various stages of execution simultaneously. This enables better throughput and improved performance compared to a single-cycle core.

A typical RISC-V pipeline core consists of several stages, with each stage dedicated to a specific operation. The classic pipeline stages include:

1. **Instruction Fetch (IF):** Fetch the instruction from memory.
2. **Instruction Decode (ID):** Decode the instruction, determining the type and the operands.
3. **Execute (EX):** Perform the actual computation or operation specified by the instruction.
4. **Memory Access (MEM):** Access memory if needed (for load/store operations).
5. **Write Back (WB):** Write the result back to the register file.

The pipeline architecture allows different instructions to be at different stages simultaneously, improving overall throughput. However, it introduces challenges such as hazards, including data hazards (dependencies between instructions), control hazards (branch instructions that affect the program flow), and structural hazards (resource conflicts).

To mitigate these hazards, additional stages and mechanisms such as forwarding, speculation, and branch prediction may be incorporated into the pipeline design.

Pipeline cores can be further categorized into different types based on the depth of the pipeline. Deeper pipelines allow for higher clock speeds but may introduce more pipeline stalls due to hazards.

Designing a RISC-V pipeline core involves a balance between pipeline depth, clock speed, and hazard handling mechanisms to achieve optimal performance. It is a more complex design compared to a single-cycle core but is closer to what is used in modern processors for achieving better performance.

# Architecture
![image](https://github.com/swapnilanand123/RISCV_Single_Cycle_Core/assets/143795450/6132359d-6c15-4237-a9cb-a2a967da5f2c)

# Waveform
https://edaplayground.com/x/7WZ2

